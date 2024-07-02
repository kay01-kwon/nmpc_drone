#pragma once

#include <iostream>
#include <random>
#include <chrono>
#include <ctime>
#include <unistd.h>
#include <limits.h>
#include <functional>

using std::mt19937;
using std::chrono::system_clock;
using std::chrono::duration_cast;

double noise(double stddev, long long seedNum);

long long get_seedNum();