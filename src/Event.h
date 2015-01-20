#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Event.h" />
///
/// <summary>
///     This module contains the definition and implementation of the Event
///     class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _EVENT_H
#define _EVENT_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_EVENT
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Event"

/**
 * This class defines the event object. An event object is used with a
 * state machine. After the state machine has initiated an operation, it
 * usaully waits for the operation to complete before moves onto the next
 * state. The event object is used for this purpose. When an event occurred,
 * a notify object is usually called. In the notify object, the event object
 * is signaled. The state machine periodically checks with the events it is
 * waiting on. If the events it is waiting on are signaled, it will advance
 * the state machine to the next state.
 */
class Event
{
private:
    bool    m_fSignaled;

public:
    // Constructor: Create an instance of the Event object.
    Event(void): m_fSignaled(false) {}

    // Destructor: Destroy an instance of the Event object.
    ~Event(void) {}

    // This function is called to set the event to signaled state.
    void SetEvent(void) {
        m_fSignaled = true;
    }

    // This function is called to clear the event.
    void ClearEvent(void) {
        m_fSignaled = false;
    }

    /**
     * This function is called to check if the event is in signaled state.
     *
     * @return Returns true if the event is in signaled state, false otherwise.
     */
    bool IsSignaled(void) {
        return m_fSignaled;
    }

};  //class Event

#endif  //ifndef _EVENT_H
