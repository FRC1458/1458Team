package frc.robot.subsystems.SwerveDrive;

import frc.robot.Constants;

public enum SwerveModuleConfig {
    FRONTLEFT (0),
    FRONTRIGHT (1),
    BACKLEFT (2),
    BACKRIGHT (3);

    public final int moduleNumber;

    public final double wheelCircumference;

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
                Constants.Swerve.FrontLeftMod.AngleMotorConfig.angleOffset
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
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveKS,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveKV,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.driveKA,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.openLoopRamp,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.closedLoopRamp,
                Constants.Swerve.FrontLeftMod.DriveMotorConfig.isInverted

            );

            canCoder = new SwerveCANCoderConfig(
                Constants.Swerve.FrontLeftMod.CanCoderConfig.canCoderID,
                Constants.Swerve.FrontLeftMod.CanCoderConfig.cancoderInvert
            );

            wheelCircumference = Constants.Swerve.FrontLeftMod.wheelCircumference;

        } else if (moduleNumber == 1) {/* Front Right */
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
                Constants.Swerve.FrontRightMod.AngleMotorConfig.angleOffset
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
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveKS,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveKV,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.driveKA,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.openLoopRamp,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.closedLoopRamp,
                Constants.Swerve.FrontRightMod.DriveMotorConfig.isInverted
            );

            canCoder = new SwerveCANCoderConfig(
                Constants.Swerve.FrontRightMod.CanCoderConfig.canCoderID,
                Constants.Swerve.FrontRightMod.CanCoderConfig.cancoderInvert
            );

            wheelCircumference = Constants.Swerve.FrontRightMod.wheelCircumference;

        } else if (moduleNumber == 2) {/* Back Left */
            angleMotor = new SwerveAngleMotorConfig(
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleMotorID,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleMotorInvert,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleNeutralMode,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleGearRatio,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleCurrentLimit,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleCurrentThreshold,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleCurrentThresholdTime,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleEnableCurrentLimit,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleKP,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleKI,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleKD,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.isInverted,
                Constants.Swerve.BackLeftMod.AngleMotorConfig.angleOffset
            );

            driveMotor = new SwerveDriveMotorConfig(
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveMotorID,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveMotorInvert,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveNeutralMode,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveGearRatio,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveCurrentLimit,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveCurrentThreshold,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveCurrentThresholdTime,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveEnableCurrentLimit,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveKP,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveKI,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveKD,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveKS,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveKV,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.driveKA,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.openLoopRamp,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.closedLoopRamp,
                Constants.Swerve.BackLeftMod.DriveMotorConfig.isInverted
            );

            canCoder = new SwerveCANCoderConfig(
                Constants.Swerve.BackLeftMod.CanCoderConfig.canCoderID,
                Constants.Swerve.BackLeftMod.CanCoderConfig.cancoderInvert
            );

            wheelCircumference = Constants.Swerve.BackLeftMod.wheelCircumference;

        } else {/* Back Right */
            angleMotor = new SwerveAngleMotorConfig(
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleMotorID,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleMotorInvert,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleNeutralMode,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleGearRatio,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleCurrentLimit,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleCurrentThreshold,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleCurrentThresholdTime,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleEnableCurrentLimit,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleKP,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleKI,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleKD,
                Constants.Swerve.BackRightMod.AngleMotorConfig.isInverted,
                Constants.Swerve.BackRightMod.AngleMotorConfig.angleOffset
            );

            driveMotor = new SwerveDriveMotorConfig(
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveMotorID,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveMotorInvert,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveNeutralMode,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveGearRatio,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveCurrentLimit,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveCurrentThreshold,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveCurrentThresholdTime,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveEnableCurrentLimit,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveKP,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveKI,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveKD,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveKS,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveKV,
                Constants.Swerve.BackRightMod.DriveMotorConfig.driveKA,
                Constants.Swerve.BackRightMod.DriveMotorConfig.openLoopRamp,
                Constants.Swerve.BackRightMod.DriveMotorConfig.closedLoopRamp,
                Constants.Swerve.BackRightMod.DriveMotorConfig.isInverted
            );

            canCoder = new SwerveCANCoderConfig(
                Constants.Swerve.BackRightMod.CanCoderConfig.canCoderID,
                Constants.Swerve.BackRightMod.CanCoderConfig.cancoderInvert
            );

            wheelCircumference = Constants.Swerve.BackRightMod.wheelCircumference;
        }
    }
}
