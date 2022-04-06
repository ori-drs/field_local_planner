#include <locally_reactive_controller/utils/timer.hpp>
#include <stdexcept>

Timer::Timer(Units units)
: timer_(Clock::now()),
  units_(units)
{
}

void Timer::setUnits(Timer::Units units)
{
  units_ = units;
}

std::string Timer::getReadableUnits() const
{
  switch(units_)
  {
    case Timer::Units::secs:
      return "s";
    case Timer::Units::millisecs:
      return "ms";
    case Timer::Units::microsecs:
      return "us";
    case Timer::Units::no_preference:
      return "not specified";
  }
}

void Timer::reset()
{
  timer_ = Clock::now();
}

double Timer::now() {
  // From https://stackoverflow.com/a/55021025/3570362
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() * 1e-6;
}

double Timer::elapsed(Timer::Units units) const
{
  Seconds dt = std::chrono::duration_cast<Seconds>(Clock::now() - timer_);

  Units requested = units;
  // If we are required to use default units, use the value saved from internal parameters
  if (units == Timer::Units::no_preference)
    requested = units_;

  // Compute the corresponding time using the chosen unit
  switch(requested)
  {
    case Timer::Units::secs:
      return dt.count();
    case Timer::Units::millisecs:
      return std::chrono::duration_cast<Millisecs>(dt).count();
    case Timer::Units::microsecs:
      return std::chrono::duration_cast<Microsecs>(dt).count();
    case Timer::Units::no_preference:
      throw std::runtime_error("Timer::elapsed cannot handle 'no_preference'");
  }
}

double Timer::lap(Timer::Units units)
{
  Seconds dt = std::chrono::duration_cast<Seconds>(Clock::now() - timer_);

  Units requested = units;
  // If we are required to use default units, use the value saved from internal parameters
  if (units == Timer::Units::no_preference)
    requested = units_;

  // Compute the corresponding time using the chosen unit
  double elapsed;
  switch(requested)
  {
    case Units::secs:
      elapsed = dt.count();
      break;
    case Units::millisecs:
      elapsed = std::chrono::duration_cast<Millisecs>(dt).count();
      break;
    case Units::microsecs:
      elapsed = std::chrono::duration_cast<Microsecs>(dt).count();
      break;
    case Units::no_preference:
      throw std::runtime_error("Timer::elapsed cannot handle 'no_preference'");
  }

  timer_ = Clock::now();
  return elapsed;
}