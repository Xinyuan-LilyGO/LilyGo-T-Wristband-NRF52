#include <nrf.h>
#include "nrf_timer.h"
#include "Timer.h"

#define OUT_PIN  25

#define nrf_timer_num   (1)
#define cc_channel_num  (0)
TimerClass timer(nrf_timer_num, cc_channel_num);


void setup() {
    Serial.setPins(0, 27);
    Serial.begin(230400);
    Serial.println("Starting...");

    pinMode(OUT_PIN, OUTPUT);
    digitalWrite(OUT_PIN, 0);

    timer.attachInterrupt(&Timer_callback, 300); // microseconds
}

void Timer_callback() {
    digitalWrite(OUT_PIN, 1); // to visualize when it was called
    digitalWrite(OUT_PIN, 0);

    timer.attachInterrupt(&Timer_callback, 300); // microseconds
}


void loop() {
    // Sleep
    __WFE(); // Enter System ON sleep mode (OFF mode would stop timers)
    __SEV(); // Make sure any pending events are cleared
    __WFE(); // More info about this sequence:
// devzone.nordicsemi.com/f/nordic-q-a/490/how-do-you-put-the-nrf51822-chip-to-sleep/2571
}

