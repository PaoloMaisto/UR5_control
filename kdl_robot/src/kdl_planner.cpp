#include "kdl_ros_control/kdl_planner.h"

KDLPlanner::KDLPlanner()
{
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

void KDLPlanner::trapezoidal_vel(double t, double tc, double &s, double &s_dot, double &s_ddot){
  // Trapezoidal profile s : [0, 1] //
  
  double s_ddot_c = -1.0 / (std::pow(tc, 2) - trajDuration_*tc);
  if (t <= tc){
    s = 0.5*s_ddot_c*std::pow(t, 2);
    s_dot = s_ddot_c*t;
    s_ddot = s_ddot_c;
  }
  else if (t <= trajDuration_-tc){
    s = s_ddot_c*tc*(t - tc/2);
    s_dot = s_ddot_c*tc;
    s_ddot = 0;
  }
  else {
    s = 1 - 0.5*s_ddot_c*std::pow(trajDuration_ - t, 2);
    s_dot = s_ddot_c*(trajDuration_ - t);
    s_ddot = -s_ddot_c;
  }

}

void KDLPlanner::cubic_polinomial(double t, double &s, double &s_dot, double &s_ddot){
  // Cubic profile s : [0, 1] // TODO: final and initial vel

  double a_3 = -2.0 / std::pow(trajDuration_, 3);
  double a_2 = 3.0 / std::pow(trajDuration_, 2);
  double a_1 = 0;
  double a_0 = 0;

  s = a_3*std::pow(t, 3) + a_2*std::pow(t, 2) + a_1*t + a_0;
  s_dot = 3*a_3*std::pow(t, 2) + 2*a_2*t + a_1;
  s_ddot = 6*a_3*t + 2*a_2;

}

void KDLPlanner::setWaypointsFrames(const std::vector<KDL::Frame>& frames,
                                   const std::vector<double>& durations)
{
    if (frames.size() < 2 || durations.size() != frames.size()-1) {
        ROS_ERROR("setWaypointsFrames: need at least 2 frames and durations size = frames-1");
        return;
    }
    frames_ = frames;
    segment_durations_ = durations;
    cumulative_time_.clear();
    cumulative_time_.push_back(0.0);
    total_duration_ = 0.0;
    for (size_t i = 0; i < segment_durations_.size(); ++i) {
        total_duration_ += segment_durations_[i];
        cumulative_time_.push_back(total_duration_);
    }
}

trajectory_point KDLPlanner::compute_multi_traj_cubic_prof_frames(const double time)
{
    trajectory_point traj; // default zeroed
    if (frames_.size() < 2) {
        ROS_WARN("compute_multi_traj_cubic_prof_frames: no frames set");
        return traj;
    }

    // clamp time
    double t = time;
    if (t <= 0.0) t = 0.0;
    if (t >= total_duration_) t = total_duration_;

    // find segment index
    size_t seg = 0;
    for (size_t i = 0; i < segment_durations_.size(); ++i) {
        if ((t >= cumulative_time_[i] && t < cumulative_time_[i+1]) ||
            (t == total_duration_ && i+1 == segment_durations_.size())) {
            seg = i;
            break;
        }
    }

    double t0 = cumulative_time_[seg];
    double Tseg = segment_durations_[seg];
    double t_local = t - t0;

    // preserve old trajDuration_
    double old_trajDuration = trajDuration_;
    trajDuration_ = Tseg;

    // compute local s, s_dot, s_ddot
    double s, s_dot, s_ddot;
    cubic_polinomial(t_local, s, s_dot, s_ddot);

    // --- translation ---
    KDL::Vector p0 = frames_[seg].p;
    KDL::Vector p1 = frames_[seg+1].p;
    KDL::Vector delta_p = p1 - p0;

    traj.pos(0) = p0.x() + s * delta_p.x();
    traj.pos(1) = p0.y() + s * delta_p.y();
    traj.pos(2) = p0.z() + s * delta_p.z();

    traj.vel(0) = s_dot * delta_p.x();
    traj.vel(1) = s_dot * delta_p.y();
    traj.vel(2) = s_dot * delta_p.z();

    traj.acc(0) = s_ddot * delta_p.x();
    traj.acc(1) = s_ddot * delta_p.y();
    traj.acc(2) = s_ddot * delta_p.z();

    // --- rotation: single-axis interpolation ---
    KDL::Rotation R0 = frames_[seg].M;
    KDL::Rotation R1 = frames_[seg+1].M;

    KDL::Rotation Rrel = R0.Inverse() * R1;

    KDL::Vector axis;
    double angle;
    angle = Rrel.GetRotAngle(axis);

    // angle ~ 0
    KDL::Rotation Rcurr;
    if (std::abs(angle) < 1e-9) {
        Rcurr = R0;
    } else {
        Rcurr = R0 * KDL::Rotation::Rot(axis, s * angle);
    }

    double qx, qy, qz, qw;
    Rcurr.GetQuaternion(qx, qy, qz, qw);
    traj.rot(0) = qx;
    traj.rot(1) = qy;
    traj.rot(2) = qz;
    traj.rot(3) = qw;

    KDL::Vector axis_world = R0 * axis;
    KDL::Vector omega = axis_world * (s_dot * angle);
    KDL::Vector alpha = axis_world * (s_ddot * angle);

    traj.ang_vel(0) = omega.x();
    traj.ang_vel(1) = omega.y();
    traj.ang_vel(2) = omega.z();

    traj.ang_acc(0) = alpha.x();
    traj.ang_acc(1) = alpha.y();
    traj.ang_acc(2) = alpha.z();

    // restore old duration
    trajDuration_ = old_trajDuration;
    return traj;
}

double KDLPlanner::getTotalDuration() const {
    return total_duration_;
}
