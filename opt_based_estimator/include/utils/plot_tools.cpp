#include "plot_tools.hpp"

PlotTool::PlotTool()
{
    size_t data_size;
    int x_tick_size, y_tick_size;
    data_size = 1000;
    x_tick_size = 5;
    y_tick_size = 4;
    
    set_data_size(data_size, x_tick_size, y_tick_size);
}

PlotTool::PlotTool(const size_t &data_size, 
const int &x_tick_size, const int &y_tick_size)
{
    set_data_size(data_size, x_tick_size, y_tick_size);
}

void PlotTool::set_keywords(const int &line_width, 
const int &label_font_size, const int &tick_font_size)
{
    set_line_width(line_width);
    set_font_size(label_font_size);
    set_font_size(tick_font_size);
}

void PlotTool::set_data_size(const int &data_size, const int &x_tick_size, const int &y_tick_size)
{
    time_vec_.reserve(data_size);

    true_data_x_.reserve(data_size);
    true_data_y_.reserve(data_size);
    true_data_z_.reserve(data_size);

    est_data_x_.reserve(data_size);
    est_data_y_.reserve(data_size);
    est_data_z_.reserve(data_size);

    x_tick_vec_.reserve(x_tick_size);
    y_tick_vec_.reserve(y_tick_size);
}

void PlotTool::add_data(const double &time, 
const vector_t &true_data, const vector_t &est_data)
{
    time_vec_.push_back(time);
    demux_and_push_back(true_data, true_data_x_, true_data_y_, true_data_z_);
    demux_and_push_back(est_data, est_data_x_, est_data_y_, est_data_z_);
}

void PlotTool::plot_data(const string &title_name, const string &y_label_name, 
const string &data1_name, const string &data2_name)
{
}

void PlotTool::set_line_width(const int &line_width)
{
}

void PlotTool::set_font_size(const int &font_size)
{
}

void PlotTool::push_back_ticks()
{
}

void PlotTool::demux_and_push_back(const vector_t &data_vec, 
vector<double> &data_x, vector<double> &data_y, vector<double> &data_z)
{
    data_x.push_back(data_vec(0));
    data_y.push_back(data_vec(1));
    data_z.push_back(data_vec(2));
}

void PlotTool::store_min_data(const vector_t &data)
{
}

void PlotTool::store_max_data(const vector_t &data)
{
}