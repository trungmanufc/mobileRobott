#include "mbed.h"
#include "millis.h"
 
static volatile uint32_t millisValue = 0;
 
static  Ticker ticker;
 
void millisTicker ()
{
    millisValue ++;
}
 
uint32_t millis ()
{
    return millisValue;
}
 
void setMillis (uint32_t theValue) {
    millisValue = theValue;
}
 
void startMillis () {
    ticker.attach (millisTicker, 0.001);    
}
 
void stopMillis () {
    ticker.detach ();
}
 