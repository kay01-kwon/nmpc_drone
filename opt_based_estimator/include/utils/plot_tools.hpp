#ifndef PLOT_TOOLS_HPP
#define PLOT_TOOLS_HPP
#include "type_definitions.h"
#include <string>
#include <map>
#include <vector>
#include <matplotlibcpp.h>

using std::string;
using std::to_string;
using std::map;
using std::pair;
using std::vector;

namespace plt = matplotlibcpp;

class PlotTool{

    public:

    PlotTool();

    PlotTool(const size_t &data_size, const int &x_tick_size, const int &y_tick_size);

    void set_keywords(const int &line_width, const int &label_font_size, const int &tick_font_size);

    void set_data_size(const int &data_size, const int &x_tick_size, const int &y_tick_size);

    void add_data(const double &time, const vector_t &true_data, const vector_t &est_data);

    void plot_data(const string &title_name, const string &y_label_name, 
    const string &data1_name, const string &data2_name);

    private:
    
    void set_line_width(const int &line_width);

    void set_font_size(const int &font_size);

    void push_back_ticks();

    void store_min_data(const vector_t &data);

    void store_max_data(const vector_t &data);

    map<string, string> line_keywords_;
    map<string, string> label_keywords_;
    map<string, string> tick_keywords_;
    map<string, string> legend_keywords_;

    vector<double> time_vec_;
    vector<double> true_data_x_, true_data_y_, true_data_z_;
    vector<double> est_data_x_, est_data_y_, est_data_z_;

    vector<double> x_tick_vec_;
    vector<double> y_tick_vec_;

    double data_max_, data_min_;

};


#endif