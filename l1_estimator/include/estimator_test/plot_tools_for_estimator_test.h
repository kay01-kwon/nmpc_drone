#ifndef PLOT_TOOLS_FOR_ESTIMATOR_TEST_H_
#define PLOT_TOOLS_FOR_ESTIMATOR_TEST_H_
#include <map>
#include <string>
#include <vector>
#include "matplotlibcpp.h"


namespace plt = matplotlibcpp;

using std::map;
using std::string;
using std::pair;
using std::to_string;

using std::vector;


map<string, string> line_keywords;
map<string, string> label_keywords;
map<string, string> ticks_keywords;
map<string, string> legend_keywords;

vector<double> x_ticks;
vector<double> y_ticks;

string y_label;
string data1_label;
string data2_label;
string data3_label;

void keywords_setup(const int &line_width, const int &font_size);

void linewidth_setup(const int& line_width);

void fontsize_setup(const int& font_size);

void ticks_setup(const double &Tf, const double &y_max, const double &y_min, 
const int &x_tick_size, const int &y_tick_size);

void plot_data(const vector<double> &time,
string& y_label_,
string& data1_label_,
string& data2_label_,
const vector<double> &data1,
const vector<double> &data2);

void plot_data(const vector<double> &time,
string &y_label_,
string &data1_label_,
string &data2_label_,
string &data3_label_,
const vector<double> &data1,
const vector<double> &data2,
const vector<double> &data3);


/**
 * @brief Set keywords for fontsize and linewidth
 * 
 * @param line_width 
 * @param font_size 
 */
void keywords_setup(const int &line_width, 
const int &font_size)
{
    linewidth_setup(line_width);
    fontsize_setup(font_size);
}

/**
 * @brief set line_width
 * 
 * @param line_width 
 */
inline void linewidth_setup(const int &line_width)
{
    line_keywords.insert(pair<string, string>
    ("linewidth",to_string(line_width))
    );

    cout << "Line width setup" << endl;
    for(const auto& line_keyword: line_keywords)
    {
        cout << line_keyword.first << ": ";
        cout << line_keyword.second << endl;
    }
}

/**
 * @brief set font size
 * 
 * @param font_size 
 */
inline void fontsize_setup(const int & font_size)
{   
    label_keywords.insert(pair<string, string>
    ("fontsize",to_string(font_size))
    );

    double tick_font_size = font_size*0.8;

    ticks_keywords.insert(pair<string, string>
    ("fontsize",to_string(tick_font_size))
    );

    legend_keywords.insert(pair<string, string>
    ("fontsize",to_string(tick_font_size))
    );

    cout << "Label keyword font setup" << endl;

    cout << label_keywords["fontsize"] << endl;

    cout << "Ticks keyword font setup" << endl;

    cout << ticks_keywords["fontsize"] << endl;

    legend_keywords.insert(std::pair<string, string>
    ("loc","upper right"));

}

/**
 * @brief set ticks for xlabel and ylabel
 * 
 * @param Tf 
 * @param y_max 
 * @param y_min 
 * @param x_ticks_size 
 * @param y_ticks_size 
 */
inline void ticks_setup(double &Tf, double y_max, double y_min, 
int x_ticks_size, int y_ticks_size)
{
    double gap;
    x_ticks.reserve(x_ticks_size);
    y_ticks.reserve(y_ticks_size);

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

    for(size_t i = 0; i < x_ticks_size; i++)
    {
        x_ticks.push_back(i*Tf/(x_ticks_size-1));
    }

    for(size_t i = 0; i < y_ticks_size; i++)
    {
        y_ticks.push_back(y_min + i*gap/(y_ticks_size-1));
    }
}

/**
 * @brief Plot data
 * 
 * @param time 
 * @param data1 reference variables
 * @param data2 state variables
 */
inline void plot_data(const vector<double> &time,
string& y_label_,
string& data1_label_,
string& data2_label_,
const vector<double> &data1,
const vector<double> &data2)
{

    line_keywords.insert(pair<string, string>
    ("color","r"));
    line_keywords.insert(pair<string, string>
    ("label", data1_label_));

    plt::plot(time, data1, line_keywords);

    line_keywords.erase("color");
    line_keywords.erase("label");

    line_keywords.insert(pair<string, string>
    ("color","violet"));
    line_keywords.insert(pair<string, string>
    ("label", data2_label_));

    line_keywords.insert(pair<string,string>
    ("linestyle","--"));

    plt::plot(time, data2, line_keywords);

    line_keywords.erase("color");
    line_keywords.erase("label");
    line_keywords.erase("linestyle");


    plt::xticks(x_ticks,ticks_keywords);
    plt::yticks(y_ticks,ticks_keywords);

    plt::xlabel("time", label_keywords);
    plt::ylabel(y_label_, label_keywords);

    plt::legend(legend_keywords);

    plt::grid(true);

    x_ticks.clear();
    y_ticks.clear();

}

inline void plot_data(const vector<double> &time, 
string &y_label_,
string &data1_label_, 
string &data2_label_, 
string &data3_label_, 
const vector<double> &data1, 
const vector<double> &data2, 
const vector<double> &data3)
{

    line_keywords.insert(pair<string, string>
    ("color","orangered"));
    line_keywords.insert(pair<string, string>
    ("label", data1_label_));

    plt::plot(time, data1, line_keywords);

    line_keywords.erase("color");
    line_keywords.erase("label");

    line_keywords.insert(pair<string, string>
    ("color","deepskyblue"));
    line_keywords.insert(pair<string, string>
    ("label", data2_label_));

    plt::plot(time, data2, line_keywords);

    line_keywords.erase("color");
    line_keywords.erase("label");


    line_keywords.insert(pair<string, string>
    ("color","limegreen"));
    line_keywords.insert(pair<string, string>
    ("label", data3_label_));
    line_keywords.insert(pair<string,string>
    ("linestyle","--"));

    plt::plot(time, data3, line_keywords);

    plt::xticks(x_ticks,ticks_keywords);
    plt::yticks(y_ticks,ticks_keywords);

    plt::xlabel("time", label_keywords);
    plt::ylabel(y_label_, label_keywords);

    plt::legend(legend_keywords);

    plt::grid(true);

    line_keywords.erase("color");
    line_keywords.erase("label");
    line_keywords.erase("linestyle");

    x_ticks.clear();
    y_ticks.clear();
}

#endif