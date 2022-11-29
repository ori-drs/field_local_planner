#pragma once

#include <chrono>
#include <string>

// From the Visual Teach and Repeat (VTR) package

class Timer {
 public:
  using Clock = std::chrono::high_resolution_clock;
  using Seconds = std::chrono::duration<double, std::ratio<1> >;
  using Millisecs = std::chrono::duration<double, std::milli>;
  using Microsecs = std::chrono::duration<double, std::micro>;

  enum Units { secs, millisecs, microsecs, no_preference };

  // Constructor
  Timer(Units units = Units::secs);

  // Set units
  void setUnits(Units units);
  // Get units for printing
  std::string getReadableUnits() const;

  // Reset timer
  void reset();

  // Get timestamp
  static double now();

  // Return elapsed time since the timer was created/reset
  double elapsed(Units units = Units::no_preference) const;

  // Return the time elapsed and resets the timer
  double lap(Units units = Units::no_preference);

 private:
  // This variable allocates the time when it was created/reset
  std::chrono::time_point<Clock> timer_;
  // Time unit
  Units units_;
};
