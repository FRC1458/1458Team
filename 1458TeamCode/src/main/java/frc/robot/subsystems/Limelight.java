// Copyright (c) FIRST and other WPILib contributors.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight subsystem. */

  public DifferentialDrivePoseEstimator m_poseEstimator;
  private Pigeon2 gyro;

  public Limelight(DifferentialDriveKinematics m_kinematics, Pigeon2 m_gyro, Encoder m_leftEncoder,
      Encoder m_rightEncoder) {
    gyro = m_gyro;
    m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(),
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05,5 * Math.PI/180),
        VecBuilder.fill(0.5, 0.5,30 * Math.PI/180));
  }

  public void LimelightPeriodic() {
    LimelightHelpers.SetRobotOrientation(
        "limelight",
        m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (mt2.tagCount > 0 && Math.abs(gyro.getRate()) > 720) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      m_poseEstimator.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
    }
  }
}

