#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"
#include <cmath>
#include "ros/ros.h"

#include <vector>
#include <kdl/frames.hpp>

struct trajectory_point {
  // Espressione della posizione
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  // Orientazione come quaternion [x, y, z, w]
  Eigen::Vector4d rot = Eigen::Vector4d(0, 0, 0, 1);  // identità

  // Velocità e accelerazione angolare
  Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d ang_acc = Eigen::Vector3d::Zero();
};

class KDLPlanner
{

public:

    KDLPlanner(double _maxVel, double _maxAcc);
    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    KDL::Trajectory* getTrajectory();

    //////////////////////////////////
    KDLPlanner();
    void cubic_polinomial(double t, double &s, double &s_dot, double &s_ddot);
    void setWaypointsFrames(const std::vector<KDL::Frame>& frames, const std::vector<double>& durations);
    trajectory_point compute_multi_traj_cubic_prof_frames(const double time);       
    double getTotalDuration() const;

private:

    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;

    //////////////////////////////////
    double trajDuration_;
    double total_duration_ = 0.0;
    Eigen::Vector3d trajInit_, trajEnd_;
    std::vector<KDL::Frame> frames_;            // waypoints (full pose)
    std::vector<double> segment_durations_;
    std::vector<double> cumulative_time_;
    trajectory_point p;

};

#endif