#ifndef READ_CONFIG_HPP_
#define READ_CONFIG_HPP_
#include "l1_estimator/type_definitions.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <fstream>

using std::string;
using std::cerr;
using YAML::Node;

class ReadConfig{

    public:

    ReadConfig() = delete;

    ReadConfig(const string& file_name);

    void get_param(inertial_param_t& inertial_param,
    aero_coeff_t& aero_coeff, double& arm_length) const;

    ~ReadConfig() = default;

    private:

    string file_name_;

    inertial_param_t inertial_param_;

    aero_coeff_t aero_coeff_;

    double arm_length_;

    void load_yaml_file();

    void get_Inertial_param_from_yaml(const Node& config);

    void get_Aero_coeff_from_yaml(const Node& config);

    void get_arm_length_from_yaml(const Node& config);

};


#endif