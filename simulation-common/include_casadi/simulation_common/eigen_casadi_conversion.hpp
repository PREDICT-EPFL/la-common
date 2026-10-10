#ifndef SRC_EIGEN_CASADI_CONVERSION_HPP
#define SRC_EIGEN_CASADI_CONVERSION_HPP

#include <casadi/casadi.hpp>
#include <Eigen/Core>

namespace eigen_casadi_conversion {

template<typename DerivedType, int DerivedRows, int DerivedCols>
inline Eigen::Matrix<DerivedType, DerivedRows, DerivedCols> toEigen(const casadi::DM &casadi_matrix)
{
    return Eigen::Matrix<DerivedType, DerivedRows, DerivedCols>::Map(
            casadi::DM::densify(casadi_matrix).nonzeros().data(), casadi_matrix.size1(), casadi_matrix.size2());
}
template<typename DerivedType, int DerivedRows, int DerivedCols>
inline void toEigen(const casadi::DM &casadi_matrix, Eigen::Matrix<DerivedType, DerivedRows, DerivedCols> &eigen_matrix)
{
    eigen_matrix = Eigen::Matrix<DerivedType, DerivedRows, DerivedCols>::Map(
            casadi::DM::densify(casadi_matrix).nonzeros().data(), casadi_matrix.size1(), casadi_matrix.size2());
}
inline Eigen::MatrixXd toEigenX(const casadi::DM &casadi_matrix)
{
    const long rows = casadi_matrix.size1();
    const long cols = casadi_matrix.size2();
    Eigen::MatrixXd eigen_matrix(rows, cols);
    eigen_matrix.setZero();
    std::memcpy(eigen_matrix.data(), casadi_matrix.ptr(), sizeof(double) * rows * cols);
    return eigen_matrix;
}
inline std::vector<double> toVec(const casadi::DM &casadi_matrix)
{
    return casadi_matrix.nonzeros();
}
inline std::vector<double> toVec(const Eigen::MatrixXd &eigenXd)
{
    return std::vector<double>(eigenXd.data(), eigenXd.size() + eigenXd.data());
}
inline casadi::DM toDM(const Eigen::MatrixXd &eigenXd)
{
    const casadi::DM dm_vec{toVec(eigenXd)};
    return reshape(dm_vec, eigenXd.rows(), eigenXd.cols());
}

template<int new_rows, int new_cols, typename DerivedType, int DerivedRows, int DerivedCols>
Eigen::Matrix<DerivedType, new_rows, new_cols>
inline reshape(const Eigen::Matrix<DerivedType, DerivedRows, DerivedCols> &mat_in)
{
    return Eigen::Map<const Eigen::Matrix<DerivedType, new_rows, new_cols>>(mat_in.data(), new_rows, new_cols);
}

} // namespace eigen_casadi_conversion

#endif //SRC_EIGEN_CASADI_CONVERSION_HPP
