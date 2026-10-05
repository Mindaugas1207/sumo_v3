#ifndef EVENT_H
#define EVENT_H

#include <cstdint>

typedef void (*EventCallback)(void*, uint32_t); 

/*! @brief Base class for all events */
class Event
{
public:
    /*! @brief Sets the callback function and context for the event
     *  @param cb The callback function to be called when the event occurs.
     *  @param context Optional context pointer to be passed to the callback function.
     */
    virtual void setCallback(EventCallback cb, void* context = nullptr) = 0;

    /*! @brief Gets the callback function for the event
     *  @return The callback function pointer.
     */
    virtual EventCallback getCallback() const = 0;

    /*! @brief Gets the context pointer for the event
     *  @return The context pointer.
     */
    virtual void* getContext() const = 0;

    /*! @brief Enables the event */
    virtual void enable() = 0;

    /*! @brief Disables the event */
    virtual void disable() = 0;
};

#endif // EVENT_H