#include <locally_reactive_controller/utils/profiler.hpp>
#include <locally_reactive_controller/utils/timer.hpp>

#include <cmath>
#include <sstream>
#include <stdlib.h>

Profiler::Event::Event(std::string name, 
                       Timer::Timer::Units units,
                       double alpha)
{
    timer_.setUnits(units);
    name_ = name;
    average_ = 0.0;
    maxDt_ = 0.0;
    minDt_ = 1e20;
    var_ = 0.0;
    alpha_ = alpha;
    isRunning_ = false;
    measurements_.reserve(Profiler::BUFFER_SIZE);
    measurementsTimestamp_.reserve(Profiler::BUFFER_SIZE);
}

Profiler::Event::~Event()
{
    // Save measurements to file
    // Get system time
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    std::string systemTime = boost::posix_time::to_iso_string(now);

    // Get home directory
    std::string homeDir = std::string(getenv("HOME"));

    // Logging file
    std::ofstream log;
    log.open(homeDir + "/profiling" + "_" + name_ + ".txt");
    
    // Write header
    log << "# Profiling for event [" << name_ << "]\n" 
        << "# Units are [" << timer_.getReadableUnits() << "]"<< std::endl;

    // Write data
    for(size_t i=0; i<measurements_.size(); i++){
        log << std::setprecision(16) << i << " " <<  measurementsTimestamp_.at(i) << " " << measurements_.at(i) << std::endl;
    }
    log.close();
}

void Profiler::Event::start()
{
    // If the event never stopped, throw a message
    if(isRunning_)
        std::cout << "[Profiler] Warning: Event [" << name_ << "] was not properly stopped. Check if you didn't miss a call to endEvent" << std::endl;
    isRunning_ = true;

    timer_.reset();
}

void Profiler::Event::end()
{
    double dt = timer_.elapsed();
    // Save measurement
    measurementsTimestamp_.push_back(Timer::now());
    measurements_.push_back(dt);
    // Update moving average
    updateStatistics(dt);

    // stop the event
    isRunning_ = false;
}

void Profiler::Event::updateStatistics(double dt)
{
    // Mean and standard deviation are computed online with
    // https://en.wikipedia.org/wiki/Moving_average#Exponentially_weighted_moving_variance_and_standard_deviation
    lastDt_ = dt;

    // Update extreme values
    maxDt_ = (dt > maxDt_) ? dt : maxDt_;
    minDt_ = (dt < minDt_) ? dt : minDt_;

    // Update difference between current measurement and last mean
    double delta = dt - average_;

    // Update average
    average_ = alpha_ * dt + (1.0 - alpha_) * average_;

    // Update variance
    var_ = (1.0 - alpha_) * (var_ + alpha_ * (delta * delta));
}

std::string Profiler::Event::name() const
{
    return name_;
}

double Profiler::Event::lastMeasurement() const
{
    return lastDt_;
}

double Profiler::Event::mean() const
{
    return average_;
}

double Profiler::Event::std() const
{
    return std::sqrt(var_);
}

double Profiler::Event::max() const
{
    return maxDt_;
}

double Profiler::Event::min() const
{
    return minDt_;
}

std::string Profiler::Event::getNiceReport()
{
    std::stringstream report;
    report << "Event: " << name() << "  ";
    report << "last_measurement[" << timer_.getReadableUnits() << "]: " << lastMeasurement() << "   ";
    report << "mean[" << timer_.getReadableUnits() << "]: " << mean() << "   ";
    report << "std [" << timer_.getReadableUnits() << "]: " << std()  << "   ";
    report << "min [" << timer_.getReadableUnits() << "]: " << min()  << "   ";
    report << "max [" << timer_.getReadableUnits() << "]: " << max();

    return report.str();
}

// Overload of the operator to allow nice printing
// TODO
//std::ostream &operator<<(std::ostream &os, const Profiler::Event &e)
//{
//    
//    return os;
//}

Profiler::Profiler(std::string name)
    : name_(name),
      enabled_(true)
{
}

//void Profiler::addEvent(std::string event, Timer::Units units)
//{
//    events_[event] = Profiler::Event(event, units);
//}

void Profiler::setEnabled(bool enabled)
{
    enabled_ = enabled;
}

const Profiler::Event &Profiler::getEvent(std::string event)
{
    // TODO check if the key exists
    return events_[event];
}

void Profiler::startEvent(std::string event, Timer::Units units)
{
    // std::lock_guard<std::mutex> guard(mutex_);

    if(!enabled_)
        return;

    // create new event if this doesn't exist
    event = name_ + "." + event;
    events_.emplace(event, Profiler::Event(event, units));

    // start measuring the event
    events_[event].start();
}

void Profiler::endEvent(std::string event)
{
    // std::lock_guard<std::mutex> guard(mutex_);

    if(!enabled_)
        return;
    
    event = name_ + "." + event;
    events_[event].end();
}

std::string Profiler::getReport()
{
    if(!enabled_)
        return "[" + name_ + "] Profiler was disabled";

    std::stringstream report;

    // Add title
    report << "[" << name_ << " - Profiling Results] \n";

    // Add events
    for(auto e : events_)
        report << e.second.getNiceReport() << "\n";

    return report.str();
}
