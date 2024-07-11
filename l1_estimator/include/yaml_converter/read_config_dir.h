#ifndef READ_CONFIG_DIR_H_
#define READ_CONFIG_DIR_H_

#include <assert.h>
#include <string>
#include <unistd.h>     // To get current Path directory

using std::string;

/**
 * @brief Get the config file name object
 * 
 * @param config_file_name 
 */
void get_config_file_name(std::string& config_file_name);

#endif