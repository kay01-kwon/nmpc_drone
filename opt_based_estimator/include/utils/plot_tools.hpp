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

    PlotTool(const size_t &size_of_data);

    void set_keywords(const int &line_width, const int &label_font_size, const int &tick_font_size);

    void add_data(const double &time, const vector_t &true_data, const vector_t &est_data);

    void plot_data(const string &y_label, const string &data1_label, const string &data2_label);

    private:
    
    void set_line_width(const int &line_width);

    void set_font_size(const int &font_size);

    void set_ticks(const int &x_tick_size, const int &y_tick_size);

    map<string, string> line_keywords_;
    map<string, string> label_keywords_;
    map<string, string> tick_keywords_;
    map<string, string> legend_keywords_;

    vector<double> theta_x_est_, theta_y_est_, theta_z_est_;
    vector<double> theta_x_exg_, theta_y_exg_, theta_z_exg_;

    vector<double> x_tick_vec_;
    vector<double> y_tick_vec_;

    size_t data_size_;

};


#endif