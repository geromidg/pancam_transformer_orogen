#include "Task.hpp"

namespace pancam_transformer {

bool Task::configureHook(void) {
    if (!TaskBase::configureHook()) return false;

    const auto& sensorToMotorRotation = _sensorToMotorRotation.rvalue();
    const auto& sensorToMotorTranslation = _sensorToMotorTranslation.rvalue();
    const auto& motorToBodyTranslation = _motorToBodyTranslation.rvalue();

    motorToBodyRotation_ = _motorToBodyRotation.get();

    Eigen::Quaterniond sensorToMotorQuaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(-M_PI / 2 + sensorToMotorRotation(2),
                    Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(sensorToMotorRotation(1),
                    Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-M_PI / 2 + sensorToMotorRotation(0),
                    Eigen::Vector3d::UnitX()));

    sensorToMotorTF_ = Eigen::Affine3d::Identity();
    sensorToMotorTF_.translation() = sensorToMotorTranslation;
    sensorToMotorTF_.linear() = sensorToMotorQuaternion.toRotationMatrix();

    motorToBodyTF_ = Eigen::Affine3d::Identity();
    motorToBodyTF_.translation() = motorToBodyTranslation;

    return true;
}

void Task::updateHook(void) {
    TaskBase::updateHook();

    calculateSensorToBodyTF();

    transformation_.setTransform(sensorToBodyTF_);
    transformation_.time = base::Time::now();
    _transformation.write(transformation_);
}

void Task::calculateSensorToBodyTF(void) {
    double motorPitch, motorYaw;

    if (_pitch.read(motorPitch) != RTT::NewData) return;
    if (_yaw.read(motorYaw) != RTT::NewData) return;

    motorPitch = (180. - motorPitch) / 2. + _motorPitchOffset.rvalue();

    Eigen::Quaterniond motorToBodyQuaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(motorYaw * M_PI/180. + motorToBodyRotation_(2),
                    Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(motorPitch * M_PI/180. + motorToBodyRotation_(1),
                    Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(motorToBodyRotation_(0),
                    Eigen::Vector3d::UnitX()));

    motorToBodyTF_.linear() = motorToBodyQuaternion.toRotationMatrix();
    sensorToBodyTF_ = motorToBodyTF_ * sensorToMotorTF_;
}

}  // namespace pancam_transformer

