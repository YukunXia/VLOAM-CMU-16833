#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/rotation.h>

#ifndef CERES_COST_FUNCTION_H
#define CERES_COST_FUNCTION_H

namespace vloam
{
struct CostFunctor33
{  // 33 means 3d - 3d observation pair
  CostFunctor33(double observed_x0, double observed_y0, double observed_z0, double observed_x1, double observed_y1,
                double observed_z1)
    :  // TODO: check if const & is necessary
    observed_x0(observed_x0)
    , observed_y0(observed_y0)
    , observed_z0(observed_z0)
    ,  // 3
    observed_x1(observed_x1)
    , observed_y1(observed_y1)
    , observed_z1(observed_z1)
  {
  }  // 3

  template <typename T>
  bool operator()(const T* const angles, const T* const t, T* residuals) const
  {
    T point3d_0to1[3];
    T point3d_0[3] = { T(observed_x0), T(observed_y0), T(observed_z0) };
    ceres::AngleAxisRotatePoint(angles, point3d_0, point3d_0to1);
    point3d_0to1[0] += t[0];
    point3d_0to1[1] += t[1];
    point3d_0to1[2] += t[2];

    residuals[0] = point3d_0to1[0] - T(observed_x1);
    residuals[1] = point3d_0to1[1] - T(observed_y1);
    residuals[2] = point3d_0to1[2] - T(observed_z1);

    return true;
  }

  static ceres::CostFunction* Create(const double observed_x0, const double observed_y0, const double observed_z0,
                                     const double observed_x1, const double observed_y1, const double observed_z1)
  {
    return (new ceres::AutoDiffCostFunction<CostFunctor33, 3, 3, 3>(
        new CostFunctor33(observed_x0, observed_y0, observed_z0, observed_x1, observed_y1, observed_z1)));
  }

  double observed_x0, observed_y0, observed_z0, observed_x1, observed_y1,
      observed_z1;  // in rectified camera 0 coordinate
                    // TODO: check if the repeated creations of cost functions will decreases the performance?
};

struct CostFunctor32
{  // 32 means 3d - 2d observation pair
  CostFunctor32(double observed_x0, double observed_y0, double observed_z0, double observed_x1_bar,
                double observed_y1_bar)
    :  // TODO: check if const & is necessary
    observed_x0(observed_x0)
    , observed_y0(observed_y0)
    , observed_z0(observed_z0)
    ,  // 3
    observed_x1_bar(observed_x1_bar)
    , observed_y1_bar(observed_y1_bar)
  {
  }  // 2

  template <typename T>
  bool operator()(const T* const angles, const T* const t, T* residuals) const
  {
    T X0[3] = { T(observed_x0), T(observed_y0), T(observed_z0) };
    T observed_x1_bar_T = T(observed_x1_bar);
    T observed_y1_bar_T = T(observed_y1_bar);

    T R_dot_X0[3];
    ceres::AngleAxisRotatePoint(angles, X0, R_dot_X0);
    R_dot_X0[0] += t[0];
    R_dot_X0[1] += t[1];
    R_dot_X0[2] += t[2];

    residuals[0] = R_dot_X0[0] - R_dot_X0[2] * observed_x1_bar_T;
    residuals[1] = R_dot_X0[1] - R_dot_X0[2] * observed_y1_bar_T;

    return true;
  }

  static ceres::CostFunction* Create(const double observed_x0, const double observed_y0, const double observed_z0,
                                     const double observed_x1_bar, const double observed_y1_bar)
  {
    return (new ceres::AutoDiffCostFunction<CostFunctor32, 2, 3, 3>(
        new CostFunctor32(observed_x0, observed_y0, observed_z0, observed_x1_bar, observed_y1_bar)));
  }

  double observed_x0, observed_y0, observed_z0, observed_x1_bar, observed_y1_bar;
  // TODO: check if the repeated creations of cost functions will decreases the performance?
};

struct CostFunctor23
{  // 23 means 2d - 3d observation pair
  CostFunctor23(double observed_x0_bar, double observed_y0_bar, double observed_x1, double observed_y1,
                double observed_z1)
    :  // TODO: check if const & is necessary
    observed_x0_bar(observed_x0_bar)
    , observed_y0_bar(observed_y0_bar)
    ,  // 2
    observed_x1(observed_x1)
    , observed_y1(observed_y1)
    , observed_z1(observed_z1)
  {
  }  // 3

  template <typename T>
  bool operator()(const T* const angles, const T* const t, T* residuals) const
  {
    T observed_x0_bar_T = T(observed_x0_bar);
    T observed_y0_bar_T = T(observed_y0_bar);

    T angles_inv[3] = { -angles[0], -angles[1], -angles[2] };
    T X1[3] = { T(observed_x1), T(observed_y1), T(observed_z1) };

    T RT_dot_X1[3];
    ceres::AngleAxisRotatePoint(angles_inv, X1, RT_dot_X1);

    T RT_dot_t[3];
    ceres::AngleAxisRotatePoint(angles_inv, t, RT_dot_t);

    T RT_dot_X1_minus_RT_dot_t[3] = { RT_dot_X1[0] - RT_dot_t[0], RT_dot_X1[1] - RT_dot_t[1],
                                      RT_dot_X1[2] - RT_dot_t[2] };

    residuals[0] = RT_dot_X1_minus_RT_dot_t[0] - RT_dot_X1_minus_RT_dot_t[2] * observed_x0_bar_T;
    residuals[1] = RT_dot_X1_minus_RT_dot_t[1] - RT_dot_X1_minus_RT_dot_t[2] * observed_y0_bar_T;

    return true;
  }

  static ceres::CostFunction* Create(const double observed_x0_bar, const double observed_y0_bar,
                                     const double observed_x1, const double observed_y1, const double observed_z1)
  {
    return (new ceres::AutoDiffCostFunction<CostFunctor23, 2, 3, 3>(
        new CostFunctor23(observed_x0_bar, observed_y0_bar, observed_x1, observed_y1, observed_z1)));
  }

  double observed_x0_bar, observed_y0_bar, observed_x1, observed_y1, observed_z1;
  // TODO: check if the repeated creations of cost functions will decreases the performance?
};

struct CostFunctor22
{  // 22 means 2d - 2d observation pair
  CostFunctor22(double observed_x0_bar, double observed_y0_bar, double observed_x1_bar, double observed_y1_bar)
    :  // TODO: check if const & is necessary
    observed_x0_bar(observed_x0_bar)
    , observed_y0_bar(observed_y0_bar)
    ,  // 2
    observed_x1_bar(observed_x1_bar)
    , observed_y1_bar(observed_y1_bar)
  {
  }  // 2

  template <typename T>
  bool operator()(const T* const angles, const T* const t, T* residuals) const
  {
    T observed_X0_bar_T[3] = { T(observed_x0_bar), T(observed_y0_bar), T(1.0) };
    T observed_X1_bar_T[3] = { T(observed_x1_bar), T(observed_y1_bar), T(1.0) };

    T observed_X0_bar_T_to1[3];
    ceres::AngleAxisRotatePoint(angles, observed_X0_bar_T, observed_X0_bar_T_to1);

    T t_cross_observed_X0_bar_T_to1[3];
    ceres::CrossProduct(t, observed_X0_bar_T_to1, t_cross_observed_X0_bar_T_to1);

    residuals[0] = ceres::DotProduct(observed_X1_bar_T, t_cross_observed_X0_bar_T_to1);

    return true;
  }

  static ceres::CostFunction* Create(const double observed_x0_bar, const double observed_y0_bar,
                                     const double observed_x1_bar, const double observed_y1_bar)
  {
    return (new ceres::AutoDiffCostFunction<CostFunctor22, 1, 3, 3>(
        new CostFunctor22(observed_x0_bar, observed_y0_bar, observed_x1_bar, observed_y1_bar)));
  }

  double observed_x0_bar, observed_y0_bar, observed_x1_bar, observed_y1_bar;
  // TODO: check if the repeated creations of cost functions will decreases the performance?
};
}  // namespace vloam

#endif