#ifndef PICO_EVENTS_H
#define PICO_EVENTS_H

#include "event.h"
#include <array>
#include <pico/stdlib.h>
#include "pico_utils.h"

/*! @brief Class representing an IO interrupt event */
class IoInterruptEvent : public Event
{
    inline static std::array<IoInterruptEvent*, 30> instances; // Assuming a maximum of 30 GPIO pins, adjust as needed
    inline static bool is_initialized = false;
    static void gpio_irq_handler(uint gpio, uint32_t events)
    {
        //utils::debug_printf("IoInterruptEvent: GPIO pin %d triggered with events 0x%x\n", gpio, events);
        if (gpio < instances.size() && instances[gpio] != nullptr && instances[gpio]->callback != nullptr)
        {
            instances[gpio]->callback(instances[gpio]->context, events);
        }
    }

    uint gpio;
    uint32_t events;
    EventCallback callback;
    void* context;
public:
    /*! @brief Default constructor for io_interrupt_event
     *  Initializes the event with no GPIO pin, no events, and no callback.
     */
    IoInterruptEvent() : gpio(0), events(0), callback(nullptr), context(nullptr) {}
    
    /*! @brief Constructor for io_interrupt_event
     *  Initializes the event with the specified GPIO pin, events, and optional callback and context.
     *  @param gpio The GPIO pin number to monitor for interrupts.
     *  @param events The GPIO interrupt events to monitor (e.g., GPIO_IRQ_EDGE_RISE, GPIO_IRQ_EDGE_FALL).
     *  @param cb Optional callback function to be called when the interrupt occurs.
     *  @param ctx Optional context pointer to be passed to the callback function.
     */
    IoInterruptEvent(uint gpio, uint32_t events, EventCallback cb = nullptr, void* ctx = nullptr)
    {
        configure(gpio, events, cb, ctx);
    }

    IoInterruptEvent(const IoInterruptEvent&) = delete;
    IoInterruptEvent& operator=(const IoInterruptEvent&) = delete;

    /*! @brief Configures this event in place (use for array elements; same parameters as the constructor) */
    void configure(uint gpio, uint32_t events, EventCallback cb = nullptr, void* ctx = nullptr)
    {
        this->gpio = gpio;
        this->events = events;
        callback = cb;
        context = ctx;
        if (!is_initialized)
        {
            utils::debug_printf("IoInterruptEvent: Initializing static instances and setting IRQ callback\n");
            instances.fill(nullptr);
            gpio_set_irq_callback(IoInterruptEvent::gpio_irq_handler);
            irq_set_enabled(IO_IRQ_BANK0, true); // gpio_set_irq_callback does not enable the NVIC line
            is_initialized = true;
        }
        if (gpio >= instances.size())
        {
            panic("GPIO pin number %d exceeds maximum supported pins for io_interrupt_event", gpio);
        }
        instances[gpio] = this;
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN);
        gpio_set_irq_enabled(gpio, events, true);
        //utils::debug_printf("IoInterruptEvent: Initialized GPIO pin %d with events 0x%08X\n", gpio, events);
    }

    /*! @brief Disables the interrupt and releases the pin; pin, events, callback and context are kept for reconfigure() */
    void deconfigure()
    {
        if (gpio < instances.size() && instances[gpio] == this)
        {
            gpio_set_irq_enabled(gpio, events, false);
            gpio_deinit(gpio);
            instances[gpio] = nullptr;
        }
    }

    /*! @brief Re-applies the configuration retained since the last configure() */
    void reconfigure()
    {
        configure(gpio, events, callback, context);
    }

    /*! @brief Sets the callback function and context for the IO interrupt event
     *  @param cb The callback function to be called when the interrupt occurs.
     *  @param ctx Optional context pointer to be passed to the callback function.
     */
    void setCallback(EventCallback cb, void* ctx = nullptr) override
    {
        callback = cb;
        context = ctx;
    }

    /*! @brief Gets the callback function for the IO interrupt event
     *  @return The callback function pointer.
     */
    EventCallback getCallback() const override
    {
        return callback;
    }

    /*! @brief Gets the context pointer for the IO interrupt event
     *  @return The context pointer.
     */
    void* getContext() const override
    {
        return context;
    }
    
    /*! @brief Enables the IO interrupt event */
    void enable() override
    {
        gpio_set_irq_enabled(gpio, events, true);
    }

    /*! @brief Disables the IO interrupt event */
    void disable() override
    {
        gpio_set_irq_enabled(gpio, events, false);
    }

    /*! @brief Destructor for io_interrupt_event
     *  Disables the interrupt and deinitializes the GPIO pin.
     */
    ~IoInterruptEvent()
    {
        if (gpio < instances.size() && instances[gpio] == this)
        {
            utils::debug_printf("IoInterruptEvent: Destroying instance for GPIO pin %d\n", gpio);
            gpio_set_irq_enabled(gpio, events, false);
            gpio_deinit(gpio);
            instances[gpio] = nullptr;
        }
    }
    
};

#endif // PICO_EVENTS_H