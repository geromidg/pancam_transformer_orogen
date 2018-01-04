#include "Task.hpp"

namespace pancam_transformer {

void Task::updateHook(void) {
    TaskBase::updateHook();

    double ptu_pitch, ptu_yaw;
    _pitch.read(ptu_pitch);
    _yaw.read(ptu_yaw);
    ptu_pitch = (180 - ptu_pitch)/2;
    ptu_pitch -= 2 * 12.5;
    ptu_pitch = ptu_pitch/180.0*M_PI;
    ptu_yaw = ptu_yaw/180.0*M_PI;

    Eigen::Quaterniond camera_orientation = Eigen::Quaterniond(
            Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
    Eigen::Vector3d camera_to_ptu(0.01, 0.25, 0.055);
    Eigen::Quaterniond ptu_orientation = Eigen::Quaterniond(
            Eigen::AngleAxisd(ptu_yaw + 0., Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ptu_pitch - 0.05, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()));
    Eigen::Vector3d ptu_to_body(0.138, -0.005, 1.286);
    Eigen::Translation3d ptu2Bd(ptu_to_body);
    Eigen::Translation3d cam2ptu(camera_to_ptu);
    Eigen::Affine3d sensorToBodyTF =
            ptu2Bd * ptu_orientation * cam2ptu * camera_orientation;

    base::samples::RigidBodyState transformation;
    transformation.setTransform(sensorToBodyTF);
    _transformation.write(transformation);
}

}  // namespace pancam_transformer

