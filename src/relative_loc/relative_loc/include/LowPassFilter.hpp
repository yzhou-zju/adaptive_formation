#pragma once

#include <mutex>
#include <math.h>
#include <Eigen/Dense>
#include <glog/logging.h>

using namespace std;
using Eigen::Quaterniond;

template <typename T>
double deltaYNorm(const T &delta_y)
{
    return delta_y.norm();
}

template <>
double deltaYNorm(const double &delta_y)
{
    return abs(delta_y);
}

template <typename T>
class LowPassFilter
{
public:
    LowPassFilter() : fc(0.0), delta_y_max(0.0), initialized(false) {}

    LowPassFilter(double cutoff_frequency, double max_delta_y)
        : fc(cutoff_frequency), delta_y_max(max_delta_y), initialized(false)
    {
        tau = 1.0 / (2 * M_PI * fc);
    }

    void input(const T &x_n, double t_n)
    {
        lock_guard<mutex> lock(y_mtx);
        if (!initialized)
        {
            y_prev = x_n;
            x_prev = x_n;
            t_prev = t_n;
            initialized = true;
            return;
        }

        double delta_t = t_n - t_prev;

        // Handle negative or zero time intervals
        if (delta_t <= 0)
        {
            y_prev = x_n;
            t_prev = t_n;
            return;
        }

        double exp_factor = exp(-delta_t / tau);

        // Compute the filtered output
        T y_n = x_prev + (y_prev - x_prev) * exp_factor;
        y_n += (x_n - x_prev) * (1 - exp_factor);

        // Apply output change threshold limit
        T delta_y = y_n - y_prev;
        double delta_y_norm = deltaYNorm(delta_y);

        if (delta_y_norm > delta_y_max)
        {
            delta_y = delta_y * (delta_y_max / delta_y_norm);
            y_n = y_prev + delta_y;
        }

        // Update state
        y_prev = y_n;
        x_prev = x_n;
        t_prev = t_n;
    }

    T output()
    {
        if (!initialized)
            LOG(ERROR) << "Accessing the output when the low-pass filter is not initialized #^# You will get a random value @_@";

        lock_guard<mutex> lock(y_mtx);
        return y_prev;
    }

    void reset()
    {
        lock_guard<mutex> lock(y_mtx);
        initialized = false;
    }

private:
    double fc;          // Cutoff frequency
    double tau;         // Time constant
    T y_prev;           // Previous output
    mutex y_mtx;        // Mutex for y_prev
    T x_prev;           // Previous input
    double t_prev;      // Previous timestamp
    double delta_y_max; // Maximum change threshold
    bool initialized;   // Initialization flag
};

// Specialization for Quaterniond
template <>
class LowPassFilter<Quaterniond>
{
public:
    LowPassFilter() : fc(0.0), delta_angle_max(0.0), initialized(false) {}

    LowPassFilter(double cutoff_frequency, double max_delta_angle)
        : fc(cutoff_frequency), delta_angle_max(max_delta_angle), initialized(false)
    {
        tau = 1.0 / (2 * M_PI * fc);
    }

    void input(const Quaterniond &x_n, double t_n)
    {
        lock_guard<mutex> lock(y_mtx);
        if (!initialized)
        {
            y_prev = x_n.normalized();
            t_prev = t_n;
            initialized = true;
            return;
        }

        double delta_t = t_n - t_prev;

        // Handle negative or zero time intervals
        if (delta_t <= 0)
        {
            y_prev = x_n.normalized();
            t_prev = t_n;
            return;
        }

        double alpha = 1 - exp(-delta_t / tau);

        // Compute the filtered output using slerp
        Quaterniond y_n = y_prev.slerp(alpha, x_n);

        // Limit the change in angle
        Quaterniond delta_q = y_prev.inverse() * y_n;
        delta_q.normalize();
        double delta_angle = 2 * acos(std::clamp(delta_q.w(), -1.0, 1.0));

        if (delta_angle > delta_angle_max)
        {
            double scale = delta_angle_max / delta_angle;
            y_n = y_prev.slerp(scale, y_n);
        }

        // Update state
        y_prev = y_n.normalized();
        t_prev = t_n;
    }

    Quaterniond output()
    {
        if (!initialized)
            LOG(ERROR) << "Accessing the output when the low-pass filter is not initialized #^# You will get a random value @_@";

        lock_guard<mutex> lock(y_mtx);
        return y_prev;
    }

    void reset()
    {
        lock_guard<mutex> lock(y_mtx);
        initialized = false;
    }

private:
    double fc;              // Cutoff frequency
    double tau;             // Time constant
    Quaterniond y_prev;     // Previous output
    mutex y_mtx;            // Mutex for y_prev
    double t_prev;          // Previous timestamp
    double delta_angle_max; // Maximum allowed angle change (radians)
    bool initialized;       // Initialization flag
};