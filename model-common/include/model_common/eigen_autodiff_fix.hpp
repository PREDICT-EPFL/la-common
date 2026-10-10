#ifndef SRC_EIGEN_AUTODIFF_FIX_HPP
#define SRC_EIGEN_AUTODIFF_FIX_HPP

namespace Eigen {

// Allow 2nd derivative without casting
template<typename DerType, typename BinOp>
struct ScalarBinaryOpTraits<AutoDiffScalar<DerType>, typename DerType::Scalar::Scalar, BinOp>
{
typedef AutoDiffScalar<DerType> ReturnType;
};

template<typename DerType, typename BinOp>
struct ScalarBinaryOpTraits<typename DerType::Scalar::Scalar, AutoDiffScalar<DerType>, BinOp>
{
typedef AutoDiffScalar<DerType> ReturnType;
};

} // end namespace Eigen

#endif //SRC__EIGEN_AUTODIFF_FIX_HPP
