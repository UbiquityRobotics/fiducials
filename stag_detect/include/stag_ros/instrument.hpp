/**
MIT License

Copyright (c) 2020 Michail Kalaitzakis, Brennan Cain (Unmanned Systems and
Robotics Lab, University of South Carolina, USA)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#pragma once
#include <chrono>
#include <numeric>
#include <vector>
#include <algorithm>
#include <iostream>

#define INSTRUMENT                                                       \
  static Instrument instrument(__PRETTY_FUNCTION__, __FILE__, __LINE__); \
  const Timer timer(instrument);

struct Instrument {
  const char* const fullname;
  int line;
  const char* const file;
  std::vector<size_t> times;

  Instrument(const char* fullname, const char* file, int line)
      : fullname(fullname), file(file), line(line) {
    std::cout << "Beginning instrumenting" << std::endl;
  }
  ~Instrument() {
    std::sort(times.begin(), times.end());
    size_t count = times.size();
    size_t sum = std::accumulate(times.begin(), times.end(), 0);

    std::cout << "---------------------------------------" << std::endl;
    printf("%s in %s:%i\n", fullname, file, line);
    printf("calls: %lu\n", times.size());
    printf("min  : %lu\n", times.front());
    printf("med  : %lu\n", times.at(count / 2));
    printf("avg  : %lu\n", sum / count);
    printf("max  : %lu\n", times.back());
    printf("total: %lu\n", sum);
    std::cout << "---------------------------------------" << std::endl;
  }
  void insert(size_t time) { times.push_back(time); }
};

struct Timer {
  Instrument& instrument;
  std::chrono::time_point<std::chrono::steady_clock> enter;
  Timer(Instrument& instrument)
      : enter(std::chrono::steady_clock::now()), instrument(instrument) {}
  ~Timer() {
    instrument.insert(std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::steady_clock::now() - enter)
                          .count());
  }
};
