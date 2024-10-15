package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class SwerveAngleMotorConfig {
    public final int angleMotorID;
    public final boolean isInverted;
    public final Rotation2d angleOffset;
    public final double angleGearRatio;
    public final TalonFXConfiguration angleFXConfig = new TalonFXConfiguration();

    public SwerveAngleMotorConfig(
        int angleMotorID,
        InvertedValue angleMotorInvert,
        NeutralModeValue angleNeutralMode,
        double angleGearRatio,
        int angleCurrentLimit,
        int angleCurrentThreshold,
        double angleCurrentThresholdTime,
        boolean angleEnableCurrentLimit,
        double angleKP,
        double angleKI,
        double angleKD,
        boolean isInverted,
        double angleOffset
    ){
        /** Swerve Angle Motor Configurations */
        this.angleMotorID = angleMotorID;
        this.isInverted = isInverted;

        this.angleOffset = Rotation2d.fromRotations((angleOffset/-360));
        this.angleGearRatio = angleGearRatio;

        /* Motor Inverts and Neutral Mode */
        angleFXConfig.MotorOutput.Inverted = angleMotorInvert;
        angleFXConfig.MotorOutput.NeutralMode = angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        angleFXConfig.Feedback.SensorToMechanismRatio = angleGearRatio;
        angleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        angleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = angleEnableCurrentLimit;
        angleFXConfig.CurrentLimits.SupplyCurrentLimit = angleCurrentLimit;
        angleFXConfig.CurrentLimits.SupplyCurrentThreshold = angleCurrentThreshold;
        angleFXConfig.CurrentLimits.SupplyTimeThreshold = angleCurrentThresholdTime;

        /* PID Config */
        angleFXConfig.Slot0.kP = angleKP;
        angleFXConfig.Slot0.kI = angleKI;
        angleFXConfig.Slot0.kD = angleKD;
    }
}