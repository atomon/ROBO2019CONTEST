#ifndef TIMER_INTERRUPT_H
#define TIMER_INTERRUPT_H

#include "move.h"
#include "Nose.h"

IntervalTimer wheel_uppdate_timer;
IntervalTimer wheel_timer;
IntervalTimer zou_timer;

void wheel_taeget_update();
void wheel_feedback();

void begin_interval_wheel()
{
    wheel_uppdate_timer.begin(wheel_taeget_update, location_interrupt_timer * 1000);      // target update   cycle : 'timer' ms
    wheel_timer.begin(wheel_feedback, wheel_interrupt_timer * 1000);   // wheel_feedback  cycle : 'wheel_interrupt_timer'  ms
}

void end_interval_wheel()
{
    wheel_uppdate_timer.end(); // end target update
    wheel_timer.end();         // end wheel_feedback
}

void wheel_taeget_update()
{
    moving.target_update();
}

void wheel_feedback()
{
    moving.feedback();
}

// zou interrupt

void zou_feedback();

void begin_interval_zou()
{
    zou_timer.begin(zou_feedback, zou_interrupt_time * 1000);              // wheel_feedback  cycle :   2000  ms
}

void end_interval_zou()
{
    zou_timer.end();         // end wheel_feedback
}

void zou_feedback()
{
    nose.feedback();
}

#endif