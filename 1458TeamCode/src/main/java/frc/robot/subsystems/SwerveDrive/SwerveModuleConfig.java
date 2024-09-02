package frc.robot.subsystems.SwerveDrive;

import frc.robot.Constants;

public enum SwerveModuleConfig {
    FRONTLEFT (0),
    FRONTRIGHT (1);

    public final int moduleNumber;

    public final SwerveAngleMotorConfig angleMotor;
    public final SwerveDriveMotorConfig driveMotor;
    public final SwerveCANCoderConfig canCoder;

    SwerveModuleConfig(int moduleNumber){
        this.moduleNumber = moduleNumber;

        // TODO: use reflection or other means to reduce the boiler plate code
        /* Front Left */
        if (moduleNumber == 0) {
            angleMotor = new SwerveAngleMotorConfig(
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleMotorID,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleMotorInvert,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleNeutralMode,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleGearRatio,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleCurrentLimit,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleCurrentThreshold,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleCurrentThresholdTime,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleEnableCurrentLimit,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleKP,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleKI,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleKD,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.isInverted,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleOffset,
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.wheelCircumference
            );

            driveMotor = new SwerveDriveMotorConfig(
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveMotorID,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveMotorInvert,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveNeutralMode,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveGearRatio,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveCurrentLimit,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveCurrentThreshold,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveCurrentThresholdTime,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveEnableCurrentLimit,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveKP,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveKI,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveKD,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.openLoopRamp,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.closedLoopRamp,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.isInverted,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.wheelCircumference

            );

            canCoder = new SwerveCANCoderConfig(
                Constants.Swerve.FrontLeftMod.CanCoderConfig.canCoderID,
                Constants.Swerve.FrontLeftMod.CanCoderConfig.cancoderInvert
            );
        } else {/* Front Right */
            angleMotor = new SwerveAngleMotorConfig(
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleMotorID,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleMotorInvert,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleNeutralMode,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleGearRatio,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleCurrentLimit,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleCurrentThreshold,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleCurrentThresholdTime,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleEnableCurrentLimit,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleKP,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleKI,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleKD,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.isInverted,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleOffset,
                Constants.Swerve.FrontRightMod.AngleMotorConfig.wheelCircumference
            );

            driveMotor = new SwerveDriveMotorConfig(
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveMotorID,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveMotorInvert,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveNeutralMode,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveGearRatio,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveCurrentLimit,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveCurrentThreshold,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveCurrentThresholdTime,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveEnableCurrentLimit,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveKP,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveKI,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveKD,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.openLoopRamp,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.closedLoopRamp,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.isInverted,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.wheelCircumference
            );

            canCoder = new SwerveCANCoderConfig(
                Constants.Swerve.FrontRightMod.CanCoderConfig.canCoderID,
                Constants.Swerve.FrontRightMod.CanCoderConfig.cancoderInvert
            );
        }
    }
}
