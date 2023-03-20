#ifndef DWBC_UTIL_H
#define DWBC_UTIL_H
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#ifndef RESOURCE_DIR
#define RESOURCE_DIR " "
#endif

using namespace Eigen;

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
std::string check_binary(const char *filename, const Matrix &matrix)
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
        return std::string(" ERROR Matrix Contains NAN");
    }
    else
    {
        if (num == 0)
        {
            return std::string(" OK!");
        }
        else if (num < 1.0E-6)
        {
            std::string ret = "Similar ( < 1.0E-6)";
            return ret + std::to_string(num);
        }
        else
        {

            std::string ret = "Error";
            return ret + std::to_string(num);
        }
    }
}
#endif