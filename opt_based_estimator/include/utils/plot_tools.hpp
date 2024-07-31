#ifndef PLOT_TOOLS_HPP
#define PLOT_TOOLS_HPP
#include "type_definitions.h"
#include <vector>
#include <matplotlibcpp.h>

using std::vector;
namespace plt = matplotlibcpp;

class PlotTool{

    public:

    PlotTool() = default;

    void set_theta(const vector_t &theta, const vector_t &theta_est);



    private:

    vector<double> theta_x_est, theta_y_est, theta_z_est;
    vector<double> theta_x_exg, theta_y_exg, theta_z_exg;

    
    
};


#endif