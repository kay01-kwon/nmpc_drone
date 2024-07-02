#include "noise_generator.hpp"

double noise(double stddev, long long seedNum)
{
    mt19937::result_type seed = seedNum;

    auto dist = std::bind(
        std::normal_distribution<double>{0, stddev},
        mt19937(seed)
        );

    return dist();
}

long long get_seedNum()
{
    auto time_stamp = system_clock::now();
    auto duration = time_stamp.time_since_epoch();
    auto milisec = 
    std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    return static_cast<long long>(milisec);
}
