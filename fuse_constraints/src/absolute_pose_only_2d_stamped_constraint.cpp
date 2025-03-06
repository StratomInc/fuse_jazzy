// Copyright Stratom Inc 2022

#include "fuse_constraints/absolute_pose_only_2d_stamped_constraint.h"

#include <fuse_constraints/normal_prior_pose_only_2d.h>
#include <pluginlib/class_list_macros.hpp>

#include <boost/serialization/export.hpp>
#include <ceres/autodiff_cost_function.h>
#include <Eigen/Dense>

#include <string>
#include <vector>

namespace fuse_constraints
{
    AbsolutePoseOnly2DStampedConstraint::AbsolutePoseOnly2DStampedConstraint(
        const std::string &source,
        const fuse_variables::Position2DStamped &position,
        const fuse_core::VectorXd &partial_mean,
        const fuse_core::MatrixXd &partial_covariance,
        const std::vector<size_t> &linear_indices) : fuse_core::Constraint(source, {position.uuid()})
    {
        size_t total_variable_size = position.size();
        size_t total_indices = linear_indices.size();

        assert(partial_mean.rows() == static_cast<int>(total_indices));
        assert(partial_covariance.rows() == static_cast<int>(total_indices));
        assert(partial_covariance.cols() == static_cast<int>(total_indices));

        // Compute the sqrt info of the cov matrix
        fuse_core::MatrixXd partial_sqrt_information = partial_covariance.inverse().llt().matrixU();

        // Assemble a mean vector and sqrt information matrix from the provided values, but in proper Variable order
        // What are we doing here?
        // The constraint equation is defined as: cost(x) = ||A * (x - b)||^2
        // If we are measuring a subset of dimensions, we only want to produce costs for the measured dimensions.
        // But the variable vectors will be full sized. We can make this all work out by creating a non-square A
        // matrix, where each row computes a cost for one measured dimensions, and the columns are in the order
        // defined by the variable.
        mean_ = fuse_core::VectorXd::Zero(total_variable_size);
        sqrt_information_ = fuse_core::MatrixXd::Zero(total_indices, total_variable_size);
        for (size_t i = 0; i < linear_indices.size(); ++i)
        {
            mean_(linear_indices[i]) = partial_mean(i);
            sqrt_information_.col(linear_indices[i]) = partial_sqrt_information.col(i);
        }
    }

    fuse_core::Matrix2d AbsolutePoseOnly2DStampedConstraint::covariance() const
    {
        // We want to compute:
        // cov = (sqrt_info' * sqrt_info)^-1
        // With some linear algebra, we can swap the transpose and the inverse.
        // cov = (sqrt_info^-1) * (sqrt_info^-1)'
        // But sqrt_info _may_ not be square. So we need to compute the pseudoinverse instead.
        // Eigen doesn't have a pseudoinverse function (for probably very legitimate reasons).
        // So we set the right hand side to identity, then solve using one of Eigen's many decompositions.
        auto I = fuse_core::MatrixXd::Identity(sqrt_information_.rows(), sqrt_information_.cols());
        fuse_core::MatrixXd pinv = sqrt_information_.colPivHouseholderQr().solve(I);
        return pinv * pinv.transpose();
    }

    void AbsolutePoseOnly2DStampedConstraint::print(std::ostream &stream) const
    {
        stream << type() << "\n"
               << "  source: " << source() << "\n"
               << "  uuid: " << uuid() << "\n"
               << "  position variable: " << variables().at(0) << "\n"
               << "  mean: " << mean().transpose() << "\n"
               << "  sqrt_info: " << sqrtInformation() << "\n";

        if (loss())
        {
            stream << "  loss: ";
            loss()->print(stream);
        }
    }
    ceres::CostFunction *AbsolutePoseOnly2DStampedConstraint::costFunction() const
    {
        return new NormalPriorPoseOnly2D(sqrt_information_, mean_);
    }

} // namespace fuse_constraints

BOOST_CLASS_EXPORT_IMPLEMENT(fuse_constraints::AbsolutePoseOnly2DStampedConstraint);
PLUGINLIB_EXPORT_CLASS(fuse_constraints::AbsolutePoseOnly2DStampedConstraint, fuse_core::Constraint);