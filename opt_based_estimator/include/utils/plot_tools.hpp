#ifndef PLOT_TOOLS_HPP
#define PLOT_TOOLS_HPP

#include <string>
#include <map>
#include <vector>
#include "type_definitions.h"

using std::map;
using std::string;
using std::pair;
using std::to_string;
using std::vector;

class PlotTool{

    public:

    PlotTool();

    PlotTool(const size_t &dim, const size_t &data_size, 
    const size_t &x_tick_size, const size_t &y_tick_size);

    void set_keywords(const double &line_width, const double &label_font_size, const double &tick_font_size);

    void add_data(const double &time, const vector_t &true_data, const vector_t &est_data);

    void plot_data(const string &title_name, const string &y_label_name, 
    const string &data1_name, const string &data2_name, const size_t index);

    private:
    
    void set_data_size(const size_t & dim, const size_t &data_size, 
    const size_t &x_tick_size, const size_t &y_tick_size);

    void set_line_width(const double &line_width);

    void set_font_size(map<string, string> &keywords, const double &font_size);

    void push_back_ticks();

    void store_min_vec(const vector_t &vec);

    void store_max_vec(const vector_t &vec);

    void set_min_data(const double& data, double &min_data);

    void set_max_data(const double& data, double &max_data);

    void compute_boundary(double &y_min, double &y_max);

    map<string, string> line_keywords_;
    map<string, string> label_keywords_;
    map<string, string> tick_keywords_;
    map<string, string> legend_keywords_;

    vector<double> time_vec_;
    vector< vector<double> > true_data_;
    vector< vector<double> > est_data_;

    vector<double> x_tick_vec_;
    vector< vector<double> > y_tick_vec_;

    vector<double> data_max_, data_min_;

    bool is_tick_pushed_back;

};


#endif