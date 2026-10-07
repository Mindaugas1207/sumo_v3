// Core 0: IMU-paced motion loop with DMA-driven I2C. Core 1: slow logic, draws into the OLED back buffer.
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

// ---------------------------------------------------------------- pins / buses
#define I2C_BUS1 i2c1 // IMU, left encoder, OLED
#define I2C_BUS1_SDA 22
#define I2C_BUS1_SCL 23
#define I2C_BUS0 i2c0 // right encoder
#define I2C_BUS0_SDA 28
#define I2C_BUS0_SCL 29
#define I2C_SPEED 400000
#define IMU_INT_PIN 21

// ---------------------------------------------------------------- devices
constexpr uint8_t LSM6DSR_ADDRESS = 0x6A;
constexpr uint8_t LSM6DSR_INT1_CTRL = 0x0D;
constexpr uint8_t LSM6DSR_CTRL1_XL = 0x10;
constexpr uint8_t LSM6DSR_CTRL2_G = 0x11;
constexpr uint8_t LSM6DSR_CTRL3_C = 0x12;
constexpr uint8_t LSM6DSR_OUTX_L_G = 0x22; // gyro XYZ then accel XYZ, 12 bytes

constexpr uint8_t MT6701_ADDRESS = 0x06;
constexpr uint8_t MT6701_ANGLE_REG = 0x03; // [13:6], then [5:0] << 2 | status

constexpr uint8_t SSD1306_ADDRESS = 0x3C;
constexpr size_t SSD1306_BUFFER_SIZE = 128 * 64 / 8;
// 18 bytes on the wire is ~0.4 ms at 400 kHz: fits in the gap after the sensor reads (833 Hz = 1.2 ms period).
constexpr size_t OLED_CHUNK = 16;

// ---------------------------------------------------------------- DMA I2C engine
constexpr uint16_t CMD_READ = I2C_IC_DATA_CMD_CMD_BITS;
constexpr uint16_t CMD_STOP = I2C_IC_DATA_CMD_STOP_BITS;
constexpr uint16_t CMD_RESTART = I2C_IC_DATA_CMD_RESTART_BITS;

struct Job
{
    uint8_t addr;
    uint8_t reg;
    uint8_t len; // <= 16 (RX FIFO depth)
    uint8_t *dst;
};

struct Bus
{
    i2c_inst_t *i2c;
    uint tx_dma, rx_dma;
    dma_channel_config tx_cfg, rx_cfg;
    const Job *jobs;
    uint8_t njobs, next;
    volatile bool busy;
    uint16_t cmd[20];
};

static Bus bus0, bus1;

static uint8_t imu_raw[12];
static uint8_t enc_l_raw[2];
static uint8_t enc_r_raw[2];

static const Job bus1_jobs[] = {
    {LSM6DSR_ADDRESS, LSM6DSR_OUTX_L_G, sizeof imu_raw, imu_raw},
    {MT6701_ADDRESS, MT6701_ANGLE_REG, sizeof enc_l_raw, enc_l_raw},
};
static const Job bus0_jobs[] = {
    {MT6701_ADDRESS, MT6701_ANGLE_REG, sizeof enc_r_raw, enc_r_raw},
};

static volatile bool frame_ready;
static volatile bool frame_deferred; // IMU IRQ arrived while an OLED chunk owned bus1
static volatile uint32_t frame_ts_us;
static volatile uint32_t overruns;

static void bus_init(Bus &b, i2c_inst_t *i2c, const Job *jobs, uint8_t njobs)
{
    b.i2c = i2c;
    b.jobs = jobs;
    b.njobs = njobs;
    b.busy = false;
    b.tx_dma = dma_claim_unused_channel(true);
    b.rx_dma = dma_claim_unused_channel(true);

    b.tx_cfg = dma_channel_get_default_config(b.tx_dma);
    channel_config_set_transfer_data_size(&b.tx_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&b.tx_cfg, true);
    channel_config_set_write_increment(&b.tx_cfg, false);
    channel_config_set_dreq(&b.tx_cfg, i2c_get_dreq(i2c, true));

    b.rx_cfg = dma_channel_get_default_config(b.rx_dma);
    channel_config_set_transfer_data_size(&b.rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&b.rx_cfg, false);
    channel_config_set_write_increment(&b.rx_cfg, true);
    channel_config_set_dreq(&b.rx_cfg, i2c_get_dreq(i2c, false));

    i2c->hw->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS;
    i2c->hw->dma_tdlr = 4;
    i2c->hw->dma_rdlr = 0;

    dma_channel_set_irq0_enabled(b.rx_dma, true); // RX completion == whole read finished
}

// Bus must be idle. TAR can only change while the controller is disabled.
static void start_transfer(Bus &b, uint8_t addr, uint nwords, uint8_t *rx_dst, uint rx_len)
{
    i2c_hw_t *hw = b.i2c->hw;
    for (int i = 0; i < 2000 && (hw->status & I2C_IC_STATUS_ACTIVITY_BITS); i++)
        tight_loop_contents();
    hw->enable = 0;
    hw->tar = addr;
    hw->enable = 1;
    (void)hw->clr_stop_det;

    if (rx_len)
        dma_channel_configure(b.rx_dma, &b.rx_cfg, rx_dst, &hw->data_cmd, rx_len, true);
    dma_channel_configure(b.tx_dma, &b.tx_cfg, &hw->data_cmd, b.cmd, nwords, true);
}

static void start_job(Bus &b)
{
    const Job &j = b.jobs[b.next++];
    b.cmd[0] = j.reg; // register pointer write
    for (uint i = 0; i < j.len; i++)
        b.cmd[1 + i] = CMD_READ;
    b.cmd[1] |= CMD_RESTART;
    b.cmd[j.len] |= CMD_STOP;
    start_transfer(b, j.addr, j.len + 1u, j.dst, j.len);
}

// Recovers from a NACK/stuck transfer: the RX DMA never completes after a TX abort.
static void bus_reset(Bus &b)
{
    dma_channel_abort(b.tx_dma);
    dma_channel_abort(b.rx_dma);
    dma_channel_acknowledge_irq0(b.rx_dma);
    b.i2c->hw->enable = 0;
    (void)b.i2c->hw->clr_tx_abrt;
    b.i2c->hw->enable = 1;
    b.busy = false;
}

static void start_frame()
{
    frame_ts_us = time_us_32();
    if (bus0.busy || bus1.busy)
    {
        overruns = overruns + 1;
        bus_reset(bus0);
        bus_reset(bus1);
    }
    bus0.next = bus1.next = 0;
    bus0.busy = true;
    bus1.busy = true;
    start_job(bus1); // IMU first, left encoder chained on completion
    start_job(bus0);
}

static void on_rx_done(Bus &b)
{
    if (b.next < b.njobs)
    {
        start_job(b);
        return;
    }
    b.busy = false;
    if (!bus0.busy && !bus1.busy)
    {
        frame_ready = true;
        __sev();
    }
}

static void dma_irq_handler()
{
    if (dma_channel_get_irq0_status(bus0.rx_dma))
    {
        dma_channel_acknowledge_irq0(bus0.rx_dma);
        on_rx_done(bus0);
    }
    if (dma_channel_get_irq0_status(bus1.rx_dma))
    {
        dma_channel_acknowledge_irq0(bus1.rx_dma);
        on_rx_done(bus1);
    }
}

// ---------------------------------------------------------------- OLED streaming
static uint8_t oled_buf[2][SSD1306_BUFFER_SIZE];
static volatile uint oled_front;     // buffer being streamed
static volatile bool oled_submitted; // core 1 finished drawing the back buffer
static uint oled_pos;
static volatile bool oled_active;

// Core 1 API
static bool oled_can_draw() { return !oled_submitted; }
static uint8_t *oled_back_buffer() { return oled_buf[oled_front ^ 1]; }
static void oled_submit()
{
    __dmb();
    oled_submitted = true;
}

static void oled_start_chunk()
{
    if (oled_pos == 0 && oled_submitted)
    {
        oled_front ^= 1;
        __dmb();
        oled_submitted = false;
    }
    const uint8_t *src = &oled_buf[oled_front][oled_pos];
    bus1.cmd[0] = 0x40; // data control byte
    for (size_t i = 0; i < OLED_CHUNK; i++)
        bus1.cmd[1 + i] = src[i];
    bus1.cmd[OLED_CHUNK] |= CMD_STOP;
    oled_pos = (oled_pos + OLED_CHUNK) % SSD1306_BUFFER_SIZE; // horizontal addressing wraps by itself
    oled_active = true;
    start_transfer(bus1, SSD1306_ADDRESS, OLED_CHUNK + 1, nullptr, 0);
}

// Called from the main loop once the sensor frame is done.
static void oled_service()
{
    uint32_t s = save_and_disable_interrupts();
    if (!oled_active && !bus1.busy && !frame_deferred)
        oled_start_chunk();
    restore_interrupts(s);
}

static void i2c1_irq_handler()
{
    (void)i2c1_hw->clr_stop_det;
    if (oled_active)
    {
        oled_active = false;
        if (frame_deferred)
        {
            frame_deferred = false;
            start_frame();
        }
    }
}

static void imu_int_callback(uint gpio, uint32_t events)
{
    if (gpio != IMU_INT_PIN)
        return;
    if (oled_active)
    {
        if (frame_deferred) // OLED transfer never finished: recover
        {
            bus_reset(bus1);
            oled_active = false;
            frame_deferred = false;
            start_frame();
        }
        else
            frame_deferred = true;
        return;
    }
    start_frame();
}

// ---------------------------------------------------------------- blocking init helpers
static void reg_write(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t d[2] = {reg, val};
    i2c_write_blocking(i2c, addr, d, 2, false);
}

static void imu_init()
{
    reg_write(I2C_BUS1, LSM6DSR_ADDRESS, LSM6DSR_CTRL3_C, 0x44);   // BDU + auto-increment
    reg_write(I2C_BUS1, LSM6DSR_ADDRESS, LSM6DSR_INT1_CTRL, 0x02); // gyro data-ready on INT1
    reg_write(I2C_BUS1, LSM6DSR_ADDRESS, LSM6DSR_CTRL1_XL, 0x78);  // 833 Hz, +-4 g
    reg_write(I2C_BUS1, LSM6DSR_ADDRESS, LSM6DSR_CTRL2_G, 0x7C);   // 833 Hz, 2000 dps
}

static void oled_init()
{
    static const uint8_t init[] = {
        0x00, // command stream
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
        0x20, 0x00, // horizontal addressing: pointer wraps over the whole frame
        0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x21, 0, 127, 0x22, 0, 7, // full-screen window
        0xAF};
    i2c_write_blocking(I2C_BUS1, SSD1306_ADDRESS, init, sizeof init, false);
}

static void i2c_pins_init(i2c_inst_t *i2c, uint sda, uint scl)
{
    i2c_init(i2c, I2C_SPEED);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

// ---------------------------------------------------------------- core 1
static void core1_main()
{
    uint8_t phase = 0;
    while (true)
    {
        if (oled_can_draw())
        {
            memset(oled_back_buffer(), phase++, SSD1306_BUFFER_SIZE); // placeholder drawing
            oled_submit();
        }
        sleep_ms(50);
    }
}

// ---------------------------------------------------------------- core 0
struct Sample
{
    int16_t gyro[3];
    int16_t accel[3];
    uint16_t enc_l; // 14 bit
    uint16_t enc_r;
    uint32_t t_us;
};

static inline uint16_t mt6701_angle(const uint8_t *r) { return (uint16_t)((r[0] << 6) | (r[1] >> 2)); }

int main()
{
    stdio_init_all();

    i2c_pins_init(I2C_BUS1, I2C_BUS1_SDA, I2C_BUS1_SCL);
    i2c_pins_init(I2C_BUS0, I2C_BUS0_SDA, I2C_BUS0_SCL);

    imu_init();
    oled_init();

    bus_init(bus1, I2C_BUS1, bus1_jobs, 2);
    bus_init(bus0, I2C_BUS0, bus0_jobs, 1);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    i2c1_hw->intr_mask = I2C_IC_INTR_MASK_M_STOP_DET_BITS; // OLED chunk completion
    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    irq_set_enabled(I2C1_IRQ, true);

    multicore_launch_core1(core1_main);

    // Latched DRDY stays high until the outputs are read; clear it so a rising edge will occur.
    uint8_t dummy[12];
    uint8_t reg = LSM6DSR_OUTX_L_G;
    i2c_write_blocking(I2C_BUS1, LSM6DSR_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_BUS1, LSM6DSR_ADDRESS, dummy, sizeof dummy, false);

    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, imu_int_callback);

    while (true)
    {
        while (!frame_ready)
            __wfe();
        frame_ready = false;

        // Next frame cannot start for ~1 ms, so the raw buffers are stable during this copy.
        Sample s;
        memcpy(s.gyro, imu_raw, 6);
        memcpy(s.accel, imu_raw + 6, 6);
        s.enc_l = mt6701_angle(enc_l_raw);
        s.enc_r = mt6701_angle(enc_r_raw);
        s.t_us = frame_ts_us;

        // TODO: sensor fusion -> motion control -> motor PWM using `s`.

        oled_service();
    }
}
