package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class SwerveDriveMotorConfig {
    public final int driveMotorID;
    public final boolean isInverted;
    public final TalonFXConfiguration motorFXConfig = new TalonFXConfiguration();

    public SwerveDriveMotorConfig(
        int driveMotorID,
        InvertedValue driveMotorInvert,
        NeutralModeValue driveNeutralMode,
        double driveGearRatio,
        int driveCurrentLimit,
        int driveCurrentThreshold,
        double driveCurrentThresholdTime,
        boolean driveEnableCurrentLimit,
        double driveKP,
        double driveKI,
        double driveKD,
        double openLoopRamp,
        double closedLoopRamp,
        boolean isInverted
    ){
        /** Swerve Drive Motor Configuration */
        this.driveMotorID = driveMotorID;
        this.isInverted = isInverted;

        /* Motor Inverts and Neutral Mode */
        motorFXConfig.MotorOutput.Inverted = driveMotorInvert;
        motorFXConfig.MotorOutput.NeutralMode = driveNeutralMode;

        /* Gear Ratio Config */
        motorFXConfig.Feedback.SensorToMechanismRatio = driveGearRatio;

        /* Current Limiting */
        motorFXConfig.CurrentLimits.SupplyCurrentLimitEnable = driveEnableCurrentLimit;
        motorFXConfig.CurrentLimits.SupplyCurrentLimit = driveCurrentLimit;
        motorFXConfig.CurrentLimits.SupplyCurrentThreshold = driveCurrentThreshold;
        motorFXConfig.CurrentLimits.SupplyTimeThreshold = driveCurrentThresholdTime;

        /* PID Config */
        motorFXConfig.Slot0.kP = driveKP;
        motorFXConfig.Slot0.kI = driveKI;
        motorFXConfig.Slot0.kD = driveKD;

        /* Open and Closed Loop Ramping */
        motorFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
        motorFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

        motorFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
        motorFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;
    }
}