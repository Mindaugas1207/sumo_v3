
#include "config.h"
#include "hardware.h"
#include "remote.h"
#include "remote_codes.h"
#include "hardware/watchdog.h"
#include "display.h"
#include "app/motion.h"

constexpr uint receiver_command_repeat_delay = 120;

void handle_line_calibration_command(void);
const char* get_ir_command_name(IrCommand command);

void remote_init(void)
{
}

void handle_remote(void)
{
    static utils::time_t last_command_time = 0;
    static IrCommand last_command = static_cast<IrCommand>(0xFF);
    IrCommand command;
    bool repeat;
    unsigned int command_raw;
    if (irReceiver.getCommand(command_raw))
    {
        command = static_cast<IrCommand>(command_raw);
        if (command == last_command)
        {
            if (utils::hasElapsed_ms(last_command_time, receiver_command_repeat_delay))
                repeat = false;
            else
                repeat = true;
        }
        else
        {
            last_command = command;
            repeat = false;
        }

#if PRINT_RECEIVER_DATA
        if (repeat)
            utils::debug_printf("Received IR command: repeat 0x%02X, %s\n", command, get_ir_command_name(command));
        else
            utils::debug_printf("Received IR command: 0x%02X, %s\n", command, get_ir_command_name(command));
#endif
        // Handle the received command as needed
        switch(command)
        {
        case IrCommand::UP:
            if (repeat) break;
            menuNavigate(-1);
            break;
        case IrCommand::DOWN:
            if (repeat) break;
            menuNavigate(1);
            break;
        case IrCommand::MENU:
            if (repeat) break;
            menuToggle();
            break;
        case IrCommand::OK:
            if (repeat) break;
            menuSelect();
            break;
        case IrCommand::VOLUP:
            if (repeat) break;
            if (!is_move_complete()) break; // prevent starting a new move before the previous one is complete
            move_linear(0.3); // Example: move forward by 10cm
            break;
        case IrCommand::VOLDOWN:
            if (repeat) break;
            if (!is_move_complete()) break; // prevent starting a new move before the previous one is complete
            move_linear(-0.3); // Example: move backward by 10cm
            break;
        case IrCommand::CHUP:
            if (repeat) break;
            if (!is_move_complete()) break; // prevent starting a new move before the previous one is complete
            move_rotational_degrees(90); // Example: rotate clockwise by 90 degrees
            break;
        case IrCommand::CHDOWN:
            if (repeat) break;
            if (!is_move_complete()) break; // prevent starting a new move before the previous one is complete
            move_rotational_degrees(-90); // Example: rotate counterclockwise by 90 degrees
            break;
        case IrCommand::MP:
            if (repeat) break;
            motion_reset();
            break;
        case IrCommand::INFO: // Toggle debug output
            if (repeat) break;
            utils::debug_enable(!utils::isDebugEnabled);
            utils::set_log_level(utils::isDebugEnabled ? utils::LogLevel::LOG_LEVEL_DEBUG : utils::LogLevel::LOG_LEVEL_INFO);
            utils::debug_printf("Debug output %s\n", utils::isDebugEnabled ? "enabled" : "disabled");
            break;
        case IrCommand::FAV: // Handle line calibration command
            if (repeat) break;
            handle_line_calibration_command();
            break;
        case IrCommand::EPG: // Handle IMU calibration command
            if (repeat) break;
            calibrate_imu();
            break;
        case IrCommand::POWER: // Handle Reboot command
            if (repeat) break;
            STATUS_Led.setBlocking(Color::Magenta());
            //active_config.configured = false;
            //save_config(true);
            utils::info_printf("Rebooting...\n");
            utils::sleep_ms(500);
            watchdog_reboot(0, 0, 0);
            //this should not be reached.
            STATUS_Led.setBlocking(Color::Black());
            break;
        default:
            break;
        }

        irReceiver.clearFIFO(); // Clear the FIFO after processing the command to avoid stale data and repeat commands on next read
        last_command_time = utils::now();
    }
}

void handle_line_calibration_command(void)
{
    STATUS_Led.blink(Color::Yellow(), Color::Black(), 100, 400);
    while (1)
    {
        STATUS_Led.update();
        uint command;
        if (!irReceiver.getCommand(command)) continue;
        if (command == IR_CODE_EXIT) break;
        if (command == IR_CODE_LEFT)
        {
            calibrate_line_sensor(front_left_line_sensor_index);
            break;
        }
        if (command == IR_CODE_RIGHT)
        {
            calibrate_line_sensor(front_right_line_sensor_index);
            break;
        }
    }
    STATUS_Led.setBlocking(Color::Black());
}

const char* get_ir_command_name(IrCommand command)
{
    switch(command)
    {
    case IrCommand::POWER: return "POWER";
    case IrCommand::AUDIO: return "AUDIO";
    case IrCommand::INFO: return "INFO";
    case IrCommand::SUBT: return "SUBT";
    case IrCommand::CH1: return "CH1";
    case IrCommand::CH2: return "CH2";
    case IrCommand::CH3: return "CH3";
    case IrCommand::CH4: return "CH4";
    case IrCommand::CH5: return "CH5";
    case IrCommand::CH6: return "CH6";
    case IrCommand::CH7: return "CH7";
    case IrCommand::CH8: return "CH8";
    case IrCommand::CH9: return "CH9";
    case IrCommand::TIME: return "TIME";
    case IrCommand::CH0: return "CH0";
    case IrCommand::CYCLE: return "CYCLE / LAST";
    case IrCommand::FAV: return "FAV";
    case IrCommand::TEXT: return "TEXT";
    case IrCommand::PVR: return "PVR / OK";
    case IrCommand::EPG: return "EPG";
    case IrCommand::MP: return "MP";
    case IrCommand::ZOOM: return "ZOOM";
    case IrCommand::MODE: return "MODE";
    case IrCommand::UP: return "UP";
    case IrCommand::LEFT: return "LEFT";
    case IrCommand::RIGHT: return "RIGHT";
    case IrCommand::DOWN: return "DOWN";
    case IrCommand::MENU: return "MENU";
    case IrCommand::TVAV: return "TVAV";
    case IrCommand::EXIT: return "EXIT";
    case IrCommand::VOLUP: return "VOLUP";
    case IrCommand::CHUP: return "CHUP";
    case IrCommand::MUTE: return "MUTE";
    case IrCommand::VOLDOWN: return "VOLDOWN";
    case IrCommand::CHDOWN: return "CHDOWN";
    default:
        return "UNKNOWN";
    }
}
