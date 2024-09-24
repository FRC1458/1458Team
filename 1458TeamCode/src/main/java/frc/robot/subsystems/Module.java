package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class Module {
    public int moduleNumber;
    private Rotation2d angleOffset;

    TalonFX mAngleMotor;
    private TalonFX mDriveMotor;

    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    /* simulation */
    // private final DifferentialDrivetrainSim mDriveSim;
    private final FlywheelSim mDriveSim;
    // private final SingleJointedArmSim mSteeringSim;

    // private final EncoderSim encoderSim;
    private final TalonFXSimState mDriveMotorSimState;
    private final TalonFXSimState mAngleMotorSimState;
    private CANcoderSimState angleEncoderSimState;

    public Module(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "CV");
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "CV");
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "CV");
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);

        if(moduleNumber == 0 || moduleNumber == 2) {
            mDriveMotor.setInverted(false);
        }

        if(moduleNumber == 0 || moduleNumber == 1 || moduleNumber == 2 || moduleNumber == 3) {
            mAngleMotor.setInverted(true);
        }
        mDriveMotor.getConfigurator().setPosition(0.0);

        // For simulation
        // encoderSim = new EncoderSim(mAngleMotor);
        mDriveMotorSimState = mDriveMotor.getSimState();
        mAngleMotorSimState = mAngleMotor.getSimState();
        angleEncoderSimState = angleEncoder.getSimState();

        final double kEncoderRadiansPerPulse = 2.0 * Math.PI / 2048;

        // mDriveSim = new DifferentialDrivetrainSim(
        //     DCMotor.getFalcon500Foc(2), // 2 CIMS on each side of the drivetrain.
        //     Constants.Swerve.driveGearRatio, // Standard AndyMark Gearing reduction.
        //     2.1, // MOI of 2.1 kg m^2 (from CAD model).
        //     26.5, // Mass of the robot is 26.5 kg.
        //     Constants.Swerve.wheelCircumference, // Robot uses 3" radius (6" diameter) wheels.
        //     0.546, // Distance between wheels is _ meters.

        //     /*
        //     * The standard deviations for measurement noise:
        //     * x and y: 0.001 m
        //     * heading: 0.001 rad
        //     * l and r velocity: 0.1 m/s
        //     * l and r position: 0.005 m
        //     */
        //     /* Uncomment the following line to add measurement noise. */
        //     null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        // );

        mDriveSim = new FlywheelSim(
            DCMotor.getFalcon500(1),
            Constants.Swerve.driveGearRatio,
            0.01,
            VecBuilder.fill(kEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
        );

        // mSteeringSim = new SingleJointedArmSim(
        //     DCMotor.getFalcon500(1),
        //     Constants.Swerve.angleGearRatio,
        //     0.001, // MOI
        //     0.0, // Length (m)
        //     Double.NEGATIVE_INFINITY, // Min angle
        //     Double.POSITIVE_INFINITY, // Max angle
        //     false, // Simulate gravity
        //     VecBuilder.fill(kEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
        // );
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
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
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
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    /** Simulate one module with naive physics model. */
    public void updateSimPeriodic() {
        // Pass the robot battery voltage to the simulated devices
        mDriveMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        mAngleMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        angleEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Simulate drive
        mDriveSim.setInput(mDriveMotorSimState.getMotorVoltage() * RobotController.getBatteryVoltage());
        // mDriveSim.setInputs(mDriveMotorSimState.getVelocity().getValue() * RobotController.getBatteryVoltage());
        mDriveSim.update(TimedRobot.kDefaultPeriod);

        double velocity = mDriveSim.getAngularVelocityRadPerSec() * Constants.Swerve.wheelCircumference / (2.0 * Math.PI);
        mDriveMotorSimState.setRawRotorPosition(velocity * Constants.Swerve.driveGearRatio);
        // angleEncoderSimState.setVelocity(velocity);

        driveDutyCycle.Output = velocity / Constants.Swerve.maxSpeed;
        mDriveMotor.setControl(driveDutyCycle);
        // else {
        //     driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
        //     driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
        //     mDriveMotor.setControl(driveVelocity);
        // }

        // // Simulate steering
        // mSteeringSim.setInput(
        //     mAngleMotor.getPosition().getValue() * RobotController.getBatteryVoltage());
        // mSteeringSim.update(TimedRobot.kDefaultPeriod);
        // mAngleMotor.setSimSensorPositionAndVelocity(
        //     mSteeringSim.getAngleRads(),
        //     mSteeringSim.getVelocityRadPerSec(),
        //     TimedRobot.kDefaultPeriod,
        //     Constants.Swerve.angleGearRatio);

        // // Update all of our sensors
        // final double leftPos = metersToRotations(mDriveSim.;
        // // This is OK, since the time base is the same
        // final double leftVel = metersToRotations(mDriveMotorSimState.getLeftVelocityMetersPerSecond());
        // final double rightPos = metersToRotations(mDriveMotorSimState.getRightPositionMeters());
        // // This is OK, since the time base is the same
        // final double rightVel = metersToRotations(mDriveMotorSimState.getRightVelocityMetersPerSecond());
        // leftSensSim.setRawPosition(leftPos);
        // leftSensSim.setVelocity(leftVel);
        // rightSensSim.setRawPosition(rightPos);
        // rightSensSim.setVelocity(rightVel);
        // leftSim.setRawRotorPosition(leftPos * this.kGearRatio);
        // leftSim.setRotorVelocity(leftVel * this.kGearRatio);
        // rightSim.setRawRotorPosition(rightPos * this.kGearRatio);
        // rightSim.setRotorVelocity(rightVel * this.kGearRatio);
    }
}