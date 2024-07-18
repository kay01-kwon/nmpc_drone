#ifndef PLOT_TOOLS_FOR_ESTIMATOR_TEST_H_
#define PLOT_TOOLS_FOR_ESTIMATOR_TEST_H_
#include <unordered_map>
#include <string>
#include <vector>
#include "matplotlibcpp.h"


namespace plt = matplotlibcpp;

using std::unordered_map;
using std::string;
using std::pair;
using std::to_string;

using std::vector;


unordered_map<string, string> line_keywords;
unordered_map<string, string> label_keywords;
unordered_map<string, string> tic_keywords;

vector<double> x_tics;
vector<double> y_tics;

void keywords_setup(const int& line_width, const int& font_size);

void linewidth_setup(const int& line_width);

void fontsize_setup(const int& font_size);

void ticks_setup(double &Tf, double &max, double &min, 
int &x_tick_size, int &y_tick_size);

void plot_two_data(const vector<double> &time, 
const vector<double> &data1,
const vector<double> &data2);

void keywords_setup(const int& line_width, const int& font_size)
{
    linewidth_setup(line_width);
    fontsize_setup(font_size);
}

inline void linewidth_setup(const int &line_width)
{
    line_keywords.insert(pair<string, string>
    ("line_width",to_string(line_width))
    );
}

inline void fontsize_setup(const int & font_size)
{
    line_keywords.insert(pair<string, string>
    ("font_size",to_string(font_size))
    );
    
    label_keywords.insert(pair<string, string>
    ("font_size",to_string(font_size))
    );

    tic_keywords.insert(pair<string, string>
    ("font_size","to_string(font_size)")
    );
}

inline void ticks_setup(double &Tf, double &y_max, double &y_min, 
int &x_tics_size, int &y_tics_size)
{
    double gap;
    x_tics.reserve(x_tics_size);
    y_tics.reserve(y_tics_size);

    if(y_min != 0)
    {
        y_min = y_min < 0 ? 1.2*y_min : 0.8*y_min;
    }
    else
    {
        y_min = -0.2;
    }

    if(y_max != 0)
    {
        y_max = y_max < 0 ? 0.8*y_max : 1.2*y_max;
    }
    else
    {
        y_max = 0.2;
    }

    gap = y_max - y_min;

    for(size_t i = 0; i < x_tics_size; i++)
    {
        x_tics.push_back(i*Tf/(x_tics_size-1));
    }

    for(size_t i = 0; i < y_tics_size; i++)
    {
        y_tics.push_back(y_min + i*gap/(y_tics_size-1));
    }
}

inline void plot_two_data(const vector<double> &time, 
const vector<double> &data1,
const vector<double> &data2)
{
    plt::plot(time, data1, line_keywords);
    plt::plot(time, data2, line_keywords);

    plt::xticks(x_tics,tic_keywords);
    plt::yticks(y_tics,tic_keywords);
    plt::grid(true);
}

#endif