#include "cnpy.h"
#include <iostream>
#include <complex>
#include <eigen3/Eigen/Dense>


std::string dirname(const std::string& s) {
    std::string::size_type last = s.find_last_of("/");
    return std::string(s.begin(), s.begin() + last);
}

std::string get_file() {
    char buffer[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", buffer, PATH_MAX);
    return std::string(buffer, (count > 0) ? count : 0);
}

std::string get_history_path(const std::string& s) {
    std::string file_path = get_file();
    std::string dir = dirname(file_path);
    return dir + "/" + s;
}

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
    string history_path = get_history_path("history.npy");
    cout << history_path << endl;

    // string history_path = get_cwd() + "/" + "history.npy";
    // get_data(loaded_history, history_path);

    // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    // cout << loaded_history.format(CleanFmt) << endl;

    return 0;
}
