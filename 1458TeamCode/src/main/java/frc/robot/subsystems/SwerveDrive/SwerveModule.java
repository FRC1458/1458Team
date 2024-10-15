package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.lib.math.Conversions;
import frc.robot.Constants;

public class SwerveModule {
    public int moduleNumber;
    private final SwerveModuleConfig swerveModuleConfig;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward;

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    /* simulation */
    // private final DifferentialDrivetrainSim mDriveSim;
    private final DCMotorSim mDriveMotorSim;
    private final DCMotorSim mAngleMotorSim;

    public SwerveModule(SwerveModuleConfig swerveModuleConfig){
        moduleNumber = swerveModuleConfig.moduleNumber;
        this.swerveModuleConfig = swerveModuleConfig;

        driveFeedForward = new SimpleMotorFeedforward(
            swerveModuleConfig.driveMotor.driveKS,
            swerveModuleConfig.driveMotor.driveKV,
            swerveModuleConfig.driveMotor.driveKA
        );

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(swerveModuleConfig.canCoder.canCoderID, "CV");
        angleEncoder.getConfigurator().apply(swerveModuleConfig.canCoder.canCoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(swerveModuleConfig.angleMotor.angleMotorID, "CV");
        mAngleMotor.getConfigurator().apply(swerveModuleConfig.angleMotor.angleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(swerveModuleConfig.driveMotor.driveMotorID, "CV");
        mDriveMotor.getConfigurator().apply(swerveModuleConfig.driveMotor.motorFXConfig);

        mDriveMotor.setInverted(swerveModuleConfig.driveMotor.isInverted);
        mAngleMotor.setInverted(swerveModuleConfig.angleMotor.isInverted);

        mDriveMotor.getConfigurator().setPosition(0.0);

        // For simulation
        mDriveMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), swerveModuleConfig.driveMotor.driveGearRatio, 0.001);
        mAngleMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), swerveModuleConfig.angleMotor.angleGearRatio, 0.001);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);

//        SmartDashboard.putNumber("Module " + moduleNumber + " Actual Angle", mAngleMotor.getPosition().getValue()); // Debugging
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, this.swerveModuleConfig.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
//        SmartDashboard.putNumber("Module " + moduleNumber + " getRotations", getCANcoder().getRotations());
//        SmartDashboard.putNumber("Module " + moduleNumber + " AngleOffset getRotations", angleOffset.getRotations());
        double absolutePosition = getCANcoder().getRotations() - this.swerveModuleConfig.angleMotor.angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), this.swerveModuleConfig.wheelCircumference),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), this.swerveModuleConfig.wheelCircumference),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public double getAngleMotorPosition(){
        // return Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()).getDegrees();
        return mAngleMotor.getPosition().getValue();
    }

    /** Simulate one module with naive physics model. */
    public void updateSimPeriodic() {
        TalonFXSimState mDriveMotorSimState = mDriveMotor.getSimState();
        TalonFXSimState mAngleMotorSimState = mAngleMotor.getSimState();
        CANcoderSimState angleEncoderSimState = angleEncoder.getSimState();

        // Pass the robot battery voltage to the simulated devices
        mDriveMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        mAngleMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        angleEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Simulate drive
        mDriveMotorSim.setInputVoltage(mDriveMotorSimState.getMotorVoltage());
        mDriveMotorSim.update(TimedRobot.kDefaultPeriod);

        double drivePosition = mDriveMotorSim.getAngularPositionRotations();
        double driveVelocity = mDriveMotorSim.getAngularVelocityRadPerSec() * this.swerveModuleConfig.wheelCircumference / (2.0 * Math.PI);
        double driveGearRatio = this.swerveModuleConfig.driveMotor.driveGearRatio;
        mDriveMotorSimState.setRawRotorPosition(drivePosition * driveGearRatio);
        mDriveMotorSimState.setRotorVelocity(driveVelocity * driveGearRatio);

        // Simulate steering
        mAngleMotorSim.setInputVoltage(mAngleMotorSimState.getMotorVoltage());
        mAngleMotorSim.update(TimedRobot.kDefaultPeriod);

        double steeringPosition = mAngleMotorSim.getAngularPositionRotations();
        double steeringVelocity = mAngleMotorSim.getAngularVelocityRadPerSec();
        double angleGearRatio = this.swerveModuleConfig.angleMotor.angleGearRatio;
        mAngleMotorSimState.setRawRotorPosition(steeringPosition * angleGearRatio);
        mAngleMotorSimState.setRotorVelocity(steeringVelocity * angleGearRatio);

        angleEncoderSimState.setRawPosition(steeringPosition * angleGearRatio);
        angleEncoderSimState.setVelocity(steeringVelocity * angleGearRatio);
    }
}