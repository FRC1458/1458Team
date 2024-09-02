package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public final class SwerveCANCoderConfig {
    public final int canCoderID;
    public final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    public SwerveCANCoderConfig(int canCoderID, SensorDirectionValue cancoderInvert){
        /** Swerve CANCoder Configuration */
        this.canCoderID = canCoderID;
        canCoderConfig.MagnetSensor.SensorDirection = cancoderInvert;
    }
}