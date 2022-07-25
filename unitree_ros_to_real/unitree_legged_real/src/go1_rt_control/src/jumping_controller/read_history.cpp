#include "cnpy.h"
#include <iostream>
#include <complex>
#include <eigen3/Eigen/Dense>

void get_data(Eigen::MatrixXd& mat, const std::string& file_name) {
    cnpy::NpyArray history = cnpy::npy_load(file_name);
    double* ptr = history.data<double>();
    int data_row = history.shape[0];
    int data_col = history.shape[1];
    mat = Eigen::MatrixXd::Map(ptr, data_row, data_col);
}

int main() {
    using namespace std;
    
    Eigen::MatrixXd loaded_history;
    get_data(loaded_history, "history.npy");

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    cout << loaded_history.format(CleanFmt) << endl;

    return 0;
}