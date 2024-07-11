#include "read_config_dir.h"

/**
 * @brief Get the config file dir object
 * 
 * @param config_file_name Destination
 * 
 * 
 * ex) 
 * string simulation_param = "simulation_model";
 * get_config_file_dir(simulation_param);
 * simulation_param will be changed to "~/catkin_ws/src/..../config/simulation_model.yaml"
 */
void get_config_file_dir(std::string &config_file_name)
{
    assert(config_file_name.size() > 0);

    string current_dir;
    current_dir = get_current_dir_name();

    config_file_name = current_dir 
                        + std::string("/config/")
                        + config_file_name
                        + std::string(".yaml");
}