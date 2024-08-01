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
    
}

void PlotTool::set_data_size(const int &data_size, const int &x_tick_size, const int &y_tick_size)
{
    data_est_.reserve(data_size);
    x_tick_vec_.reserve(x_tick_size);
    y_tick_vec_.reserve(y_tick_size);
}

void PlotTool::add_data(const double &time, 
const vector_t &true_data, const vector_t &est_data)
{
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

void PlotTool::store_min_data(const vector_t &data)
{
}

void PlotTool::store_max_data(const vector_t &data)
{
}