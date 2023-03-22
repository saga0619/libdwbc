#ifndef DWBC_TEST_UTIL_H
#define DWBC_TEST_UTIL_H
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#ifndef RESOURCE_DIR
#define RESOURCE_DIR " "
#endif

using namespace Eigen;

#define COMPARISON_EPS 1.0E-6

template <class Matrix>
void write_binary(const char *filename, const Matrix &matrix)
{
    std::string r_path = RESOURCE_DIR;

    std::string t_path = r_path + filename;

    std::ofstream out(t_path, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
    out.write((char *)(&rows), sizeof(typename Matrix::Index));
    out.write((char *)(&cols), sizeof(typename Matrix::Index));
    out.write((char *)matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
    out.close();
}
template <class Matrix>
int check_binary(const char *filename, const Matrix &matrix)
{
    std::string r_path = RESOURCE_DIR;

    std::string t_path = r_path + filename;

    Matrix mat_get;
    std::ifstream in(t_path, std::ios::in | std::ios::binary);
    typename Matrix::Index rows = 0, cols = 0;
    in.read((char *)(&rows), sizeof(typename Matrix::Index));
    in.read((char *)(&cols), sizeof(typename Matrix::Index));
    mat_get.resize(rows, cols);
    in.read((char *)mat_get.data(), rows * cols * sizeof(typename Matrix::Scalar));
    in.close();

    double num = (mat_get - matrix).norm();

    if (matrix.hasNaN())
    {
        return -1; //std::string(" ERROR Matrix Contains NAN");
    }
    else
    {
        if (num == 0)
        {
            return 1; //std::string(" OK!");
        }
        else if (num < COMPARISON_EPS)
        {
            // std::string ret = "Similar ( < 1.0E-6)";
            return 2; //ret + std::to_string(num);
        }
        else
        {

            // std::string ret = "Error";
            return 0; // ret + std::to_string(num);
        }
    }
}


#endif