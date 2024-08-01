#include "plot_tools.hpp"
// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;

PlotTool::PlotTool()
{
    size_t dim, data_size;
    int x_tick_size, y_tick_size;
    dim = 3;
    data_size = 1000;
    x_tick_size = 5;
    y_tick_size = 4;
    
    set_data_size(dim, data_size, x_tick_size, y_tick_size);
}

PlotTool::PlotTool(const size_t &dim, const size_t &data_size, 
const size_t &x_tick_size, const size_t &y_tick_size)
{
    set_data_size(dim, data_size, x_tick_size, y_tick_size);
}

void PlotTool::set_keywords(const double &line_width, 
const double &label_font_size, const double &tick_font_size)
{
    assert(line_width > 0);
    assert(label_font_size > 0);
    assert(tick_font_size > 0);

    set_line_width(line_width);

    set_font_size(label_keywords_, label_font_size);

    set_font_size(tick_keywords_, tick_font_size);
}

void PlotTool::add_data(const double &time, 
const vector_t &true_data, const vector_t &est_data)
{
    time_vec_.push_back(time);

    size_t true_data_dim = true_data_.capacity();
    size_t est_data_dim = est_data_.capacity();

    assert(true_data_dim == est_data_dim);

    for(size_t i = 0; i < true_data_dim; i++)
    {
        true_data_[i].push_back(true_data(i));
        est_data_[i].push_back(est_data(i));
    }

    store_max_vec(est_data);
    store_min_vec(est_data);

}

void PlotTool::plot_data(const string &title_name, const string &y_label_name, 
const string &data1_name, const string &data2_name)
{
}

void PlotTool::set_data_size(const size_t &dim, const size_t &data_size, 
const size_t &x_tick_size, const size_t &y_tick_size)
{
    assert(dim != 0);
    assert(data_size != 0);
    
    time_vec_.reserve(data_size);

    true_data_.reserve(dim);
    est_data_.reserve(dim);

    data_max_.reserve(dim);
    data_min_.reserve(dim);

    x_tick_vec_.reserve(x_tick_size);

    y_tick_vec_.reserve(dim);

    for(size_t i = 0; i < true_data_.capacity(); i++)
    {
        true_data_[i].reserve(data_size);
        est_data_[i].reserve(data_size);
        y_tick_vec_[i].reserve(y_tick_size);
    }

}

void PlotTool::set_line_width(const double &line_width)
{
    line_keywords_.insert(pair<string, string>
    ("linewidth",to_string(line_width))
    );
}

void PlotTool::set_font_size(map<string, string> &keywords, const double &font_size)
{
    keywords.insert(pair<string, string>
    ("fontsize",to_string(font_size))
    );
}

void PlotTool::push_back_ticks()
{
    double gap[3];

    for(size_t i = 0; i < est_data_.capacity(); i++)
    {
        compute_boundary(data_min_[i], data_max_[i]);

        gap[i] = data_max_[i] - data_min_[i];
    }

}

void PlotTool::store_min_vec(const vector_t &vec)
{
    for(size_t i = 0; i < 3; i++)
        set_min_data(vec(i), data_max_[i]);
}

void PlotTool::store_max_vec(const vector_t &vec)
{
    for(size_t i = 0; i < 3; i++)
        set_max_data(vec(i), data_min_[i]);
}

void PlotTool::set_max_data(const double &data, double &max_data)
{
    if(max_data < data)
        max_data = data;
}

void PlotTool::set_min_data(const double &data, double &min_data)
{
    if(min_data > data)
        min_data = data;
}

void PlotTool::compute_boundary(double &y_min, double &y_max)
{
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
}