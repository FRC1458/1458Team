package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.lib.util.Conversions;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class Module extends Subsystem {  // Supports looper
    public int moduleNumber;
    private Rotation2d angleOffset;

     TalonFX mAngleMotor;
     TalonFX mDriveMotor;
     CANcoder angleEncoder;

     /*

     private static Module m_Instance;

    public static Module getInstance() {
		if (m_Instance == null) {
			m_Instance = new Module();
		}
		return m_Instance;
	}
    */

    //Table that gets updated every 2 ms

    private mPeriodicIO mPeriodicIO = new mPeriodicIO();

    public static class mPeriodicIO {
		// Inputs
		public double timestamp = 0.0;
		public double rotationPosition = 0.0;
		public double rotationVelocity = 0.0;
		public double drivePosition = 0.0;
		public double driveVelocity = 0.0;

		// Outputs
		public double targetVelocity = 0.0;
		public ControlRequest rotationDemand;
		public ControlRequest driveDemand;
	}

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    /* simulation */
    private final DCMotorSim mDriveMotorSim;
    private final DCMotorSim mAngleMotorSim;

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
        mDriveMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.Swerve.driveGearRatio, 0.001);
        mAngleMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.Swerve.angleGearRatio, 0.001);
    }

	@Override
	public synchronized void readPeriodicInputs() {
		mPeriodicIO.timestamp = Timer.getFPGATimestamp();
		refreshSignals();
	}


    public synchronized void refreshSignals() {
		mPeriodicIO.rotationVelocity = mAngleMotor.getRotorVelocity().getValue();
		mPeriodicIO.driveVelocity = mDriveMotor.getRotorVelocity().getValue();

		mPeriodicIO.rotationPosition = BaseStatusSignal.getLatencyCompensatedValue(
				mAngleMotor.getRotorPosition(), mAngleMotor.getRotorVelocity());
		mPeriodicIO.drivePosition = mDriveMotor.getRotorPosition().getValueAsDouble();

        DisplayToDahboard();

	}

    @Override
	public synchronized void writePeriodicOutputs() {

        //mAngleMotor.getRotorVelocity().getValue();
        //mAngleMotor.setControl(nu
	}
    //Sample Push


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
        SmartDashboard.putNumber("Module " + moduleNumber + " getRotations", getCANcoder().getRotations());
        SmartDashboard.putNumber("Module " + moduleNumber + " AngleOffset getRotations", angleOffset.getRotations());
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

    public void DisplayToDahboard() {
        SmartDashboard.putNumber("Module " + moduleNumber + " getRotations", getCANcoder().getRotations());
        SmartDashboard.putNumber("Module " + moduleNumber + " AngleOffset getRotations", angleOffset.getRotations());
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
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
        double driveVelocity = mDriveMotorSim.getAngularVelocityRadPerSec() * Constants.Swerve.wheelCircumference / (2.0 * Math.PI);
        mDriveMotorSimState.setRawRotorPosition(drivePosition * Constants.Swerve.driveGearRatio);
        mDriveMotorSimState.setRotorVelocity(driveVelocity * Constants.Swerve.driveGearRatio);

        // Simulate steering
        mAngleMotorSim.setInputVoltage(mAngleMotorSimState.getMotorVoltage());
        mAngleMotorSim.update(TimedRobot.kDefaultPeriod);

        double steeringPosition = mAngleMotorSim.getAngularPositionRotations();
        double steeringVelocity = mAngleMotorSim.getAngularVelocityRadPerSec();
        mAngleMotorSimState.setRawRotorPosition(steeringPosition * Constants.Swerve.angleGearRatio);
        mAngleMotorSimState.setRotorVelocity(steeringVelocity * Constants.Swerve.angleGearRatio);

        angleEncoderSimState.setRawPosition(steeringPosition * Constants.Swerve.angleGearRatio);
        angleEncoderSimState.setVelocity(steeringVelocity * Constants.Swerve.angleGearRatio);
    }
}