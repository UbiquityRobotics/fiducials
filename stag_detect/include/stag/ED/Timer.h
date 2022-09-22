#ifndef TIMER_H
#define TIMER_H

#include <windows.h>

class Timer {
 private:
  __int64 freq, tStart, tStop;

 public:
  Timer() {
    // Get the frequency of the hi-res timer
    QueryPerformanceFrequency((LARGE_INTEGER*)&freq);
  }  // end-TimerClass

  void Start() {
    // Use hi-res timer
    QueryPerformanceCounter((LARGE_INTEGER*)&tStart);
  }  // end-Start

  void Stop() {
    // Perform operations that require timing
    QueryPerformanceCounter((LARGE_INTEGER*)&tStop);
  }  // end-Stop

  // Returns time in milliseconds
  double ElapsedTime() {
    // Calculate time difference in milliseconds
    return ((double)(tStop - tStart) / (double)freq) * 1e3;
  }  // end-Elapsed
};

#endif