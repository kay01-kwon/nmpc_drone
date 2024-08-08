#include "optimization_method/optimization_node.hpp"

int main(int argc, char** argv)
{
    cout << "Test" << endl;
    OptMethodName opt_method_name;
    opt_method_name = OptMethodName::GradDesc;
    OptNode<double> opt_node(opt_method_name);
}