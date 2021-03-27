#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/loss_function.h>

#ifndef CERES_COST_FUNCTION_H
#define CERES_COST_FUNCTION_H

namespace vloam {
    struct CostFunctor33 {  // 33 means 3d - 3d observation pair
        CostFunctor33(double observed_x0, double observed_y0, double observed_z0, double observed_x1, double observed_y1, double observed_z1) : // TODO: check if const & is necessary
            observed_x0(observed_x0), observed_y0(observed_y0), observed_z0(observed_z0), // 3
            observed_x1(observed_x1), observed_y1(observed_y1), observed_z1(observed_z1) {} // 3

        template <typename T>
        bool operator()(const T* const angles, const T* const t, T* residuals) const {
            T point3d_0to1[3];
            T point3d_0[3] = {T(observed_x0), T(observed_y0), T(observed_z0)};
            ceres::AngleAxisRotatePoint(angles, point3d_0, point3d_0to1);
            point3d_0to1[0] += t[0];
            point3d_0to1[1] += t[1];
            point3d_0to1[2] += t[2];

            residuals[0] = point3d_0to1[0] - T(observed_x1);
            residuals[1] = point3d_0to1[1] - T(observed_y1);
            residuals[2] = point3d_0to1[2] - T(observed_z1);

            return true;
        }
        
        static ceres::CostFunction* Create(const double observed_x0,
                                        const double observed_y0,
                                        const double observed_z0,
                                        const double observed_x1,
                                        const double observed_y1,
                                        const double observed_z1) {
            return (new ceres::AutoDiffCostFunction<CostFunctor33, 3, 3, 3>(
                new CostFunctor33(observed_x0, observed_y0, observed_z0, observed_x1, observed_y1, observed_z1)
            ));
        }

        double observed_x0, observed_y0, observed_z0, observed_x1, observed_y1, observed_z1; // in rectified camera 0 coordinate
        // TODO: check if the repeated creations of cost functions will decreases the performance?
    };

    struct CostFunctor32 {  // 32 means 3d - 2d observation pair
        CostFunctor32(double observed_x0, double observed_y0, double observed_z0, double observed_x1_bar, double observed_y1_bar) : // TODO: check if const & is necessary
            observed_x0(observed_x0), observed_y0(observed_y0), observed_z0(observed_z0), // 3
            observed_x1_bar(observed_x1_bar), observed_y1_bar(observed_y1_bar) {} // 2

        template <typename T>
        bool operator()(const T* const angles, const T* const t, T* residuals) const {
            T point3d_0[3] = {T(observed_x0), T(observed_y0), T(observed_z0)};
            T observed_x1_bar_T = T(observed_x1_bar);
            T observed_y1_bar_T = T(observed_y1_bar);

            T R[9];
            ceres::AngleAxisToRotationMatrix(angles, R);

            T R1_minus_x1bar_R3[3] = {
                R[0] - observed_x1_bar_T * R[6], 
                R[1] - observed_x1_bar_T * R[7], 
                R[2] - observed_x1_bar_T * R[8]};
            T R2_minus_y1bar_R3[3] = {
                R[3] - observed_x1_bar_T * R[6], 
                R[4] - observed_x1_bar_T * R[7], 
                R[5] - observed_x1_bar_T * R[8]};

            residuals[0] = 
                            R1_minus_x1bar_R3[0] * point3d_0[0] + 
                            R1_minus_x1bar_R3[1] * point3d_0[1] + 
                            R1_minus_x1bar_R3[2] * point3d_0[2] + 
                            t[0] - observed_x1_bar_T * t[2];
            residuals[1] = 
                            R2_minus_y1bar_R3[0] * point3d_0[0] + 
                            R2_minus_y1bar_R3[1] * point3d_0[1] + 
                            R2_minus_y1bar_R3[2] * point3d_0[2] + 
                            t[1] - observed_y1_bar_T * t[2];

            return true;
        }
        
        static ceres::CostFunction* Create(const double observed_x0,
                                        const double observed_y0,
                                        const double observed_z0,
                                        const double observed_x1_bar,
                                        const double observed_y1_bar) {
            return (new ceres::AutoDiffCostFunction<CostFunctor32, 2, 3, 3>(
                new CostFunctor32(observed_x0, observed_y0, observed_z0, observed_x1_bar, observed_y1_bar)
            ));
        }

        double observed_x0, observed_y0, observed_z0, observed_x1_bar, observed_y1_bar;
        // TODO: check if the repeated creations of cost functions will decreases the performance?
    };

    struct CostFunctor23 {  // 23 means 2d - 3d observation pair
        CostFunctor23(double observed_x0_bar, double observed_y0_bar, double observed_x1, double observed_y1, double observed_z1) : // TODO: check if const & is necessary
            observed_x0_bar(observed_x0_bar), observed_y0_bar(observed_y0_bar), // 2
            observed_x1(observed_x1), observed_y1(observed_y1), observed_z1(observed_z1) {} // 3

        template <typename T>
        bool operator()(const T* const angles, const T* const t, T* residuals) const {
            T point3d_1[3] = {T(observed_x1), T(observed_y1), T(observed_z1)};
            T observed_x0_bar_T = T(observed_x0_bar);
            T observed_y0_bar_T = T(observed_y0_bar);

            T R[9];
            ceres::AngleAxisToRotationMatrix(angles, R);
            // T R_transpose[9] = { R[0], R[3], R[6],
            //                      R[1], R[4], R[7], 
            //                      R[2], R[5], R[8]};
            T R_transpose_dot_t[3] = {  R[0]*t[0] + R[3]*t[1] + R[6]*t[2], 
                                        R[1]*t[0] + R[4]*t[1] + R[7]*t[2], 
                                        R[2]*t[0] + R[5]*t[1] + R[8]*t[2]};

            T RT1_minus_x0bar_RT3[3] = {
                R[0] - observed_x0_bar_T * R[2], 
                R[3] - observed_x0_bar_T * R[5], 
                R[6] - observed_x0_bar_T * R[8]};
            T RT2_minus_y0bar_RT3[3] = {
                R[1] - observed_y0_bar_T * R[2], 
                R[4] - observed_y0_bar_T * R[5], 
                R[7] - observed_y0_bar_T * R[8]};

            residuals[0] = 
                            RT1_minus_x0bar_RT3[0] * point3d_1[0] + 
                            RT1_minus_x0bar_RT3[1] * point3d_1[1] + 
                            RT1_minus_x0bar_RT3[2] * point3d_1[2] - 
                            R_transpose_dot_t[0] + observed_y0_bar_T * R_transpose_dot_t[2];
            residuals[1] = 
                            RT2_minus_y0bar_RT3[0] * point3d_1[0] + 
                            RT2_minus_y0bar_RT3[1] * point3d_1[1] + 
                            RT2_minus_y0bar_RT3[2] * point3d_1[2] - 
                            R_transpose_dot_t[1] + observed_y0_bar_T * R_transpose_dot_t[2];

            return true;
        }
        
        static ceres::CostFunction* Create(const double observed_x0_bar,
                                        const double observed_y0_bar,
                                        const double observed_x1,
                                        const double observed_y1,
                                        const double observed_z1 ) {
            return (new ceres::AutoDiffCostFunction<CostFunctor23, 2, 3, 3>(
                new CostFunctor23(observed_x0_bar, observed_y0_bar, observed_x1, observed_y1, observed_z1)
            ));
        }

        double observed_x0_bar, observed_y0_bar, observed_x1, observed_y1, observed_z1; 
        // TODO: check if the repeated creations of cost functions will decreases the performance?
    };

    struct CostFunctor22 {  // 22 means 2d - 2d observation pair
        CostFunctor22(double observed_x0_bar, double observed_y0_bar, double observed_x1_bar, double observed_y1_bar) : // TODO: check if const & is necessary
            observed_x0_bar(observed_x0_bar), observed_y0_bar(observed_y0_bar), // 2
            observed_x1_bar(observed_x1_bar), observed_y1_bar(observed_y1_bar) {} // 2

        template <typename T>
        bool operator()(const T* const angles, const T* const t, T* residuals) const {
            T observed_x0_bar_T = T(observed_x0_bar);
            T observed_y0_bar_T = T(observed_y0_bar);
            T observed_x1_bar_T = T(observed_x1_bar);
            T observed_y1_bar_T = T(observed_y1_bar);

            T R[9];
            ceres::AngleAxisToRotationMatrix(angles, R);

            T x1_bar_dot_skew_t[3] = {
                -observed_y1_bar_T*t[2] + t[1], 
                observed_x1_bar_T*t[2] - t[0],
                -observed_x1_bar_T*t[1] + observed_y1_bar_T*t[0]
            };

            T x1_bar_dot_skew_t_dot_R[3] = {
                x1_bar_dot_skew_t[0]*R[0] + x1_bar_dot_skew_t[1]*R[3] + x1_bar_dot_skew_t[2]*R[6],
                x1_bar_dot_skew_t[0]*R[1] + x1_bar_dot_skew_t[1]*R[4] + x1_bar_dot_skew_t[2]*R[7],
                x1_bar_dot_skew_t[0]*R[2] + x1_bar_dot_skew_t[1]*R[5] + x1_bar_dot_skew_t[2]*R[8]
            };

            residuals[0] =  x1_bar_dot_skew_t_dot_R[0]*observed_x0_bar_T + 
                            x1_bar_dot_skew_t_dot_R[1]*observed_y0_bar_T +
                            x1_bar_dot_skew_t_dot_R[2];

            return true;
        }
        
        static ceres::CostFunction* Create(const double observed_x0_bar,
                                        const double observed_y0_bar,
                                        const double observed_x1_bar,
                                        const double observed_y1_bar) {
            return (new ceres::AutoDiffCostFunction<CostFunctor22, 1, 3, 3>(
                new CostFunctor22(observed_x0_bar, observed_y0_bar, observed_x1_bar, observed_y1_bar)
            ));
        }

        double observed_x0_bar, observed_y0_bar, observed_x1_bar, observed_y1_bar;
        // TODO: check if the repeated creations of cost functions will decreases the performance?
    };
}

#endif