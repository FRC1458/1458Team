package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Shooter mInstance;
  private PeriodicIO mPeriodicIO;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private CANSparkFlex mLeftShooterMotor;
  private CANSparkFlex mRightShooterMotor;

  private SparkPIDController mLeftShooterPID;
  private SparkPIDController mRightShooterPID;

  private RelativeEncoder mLeftShooterEncoder;
  private RelativeEncoder mRightShooterEncoder;

  private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private Shooter() {
    //super("Shooter");

    mPeriodicIO = new PeriodicIO();

    mLeftShooterMotor = new CANSparkFlex(Constants.Shooter.kShooterLeftMotorId, MotorType.kBrushless);
    mRightShooterMotor = new CANSparkFlex(Constants.Shooter.kShooterRightMotorId, MotorType.kBrushless);
    mLeftShooterMotor.restoreFactoryDefaults();
    mRightShooterMotor.restoreFactoryDefaults();

    mLeftShooterPID = mLeftShooterMotor.getPIDController();
    mLeftShooterPID.setP(Constants.Shooter.kShooterP);
    mLeftShooterPID.setI(Constants.Shooter.kShooterI);
    mLeftShooterPID.setD(Constants.Shooter.kShooterD);
    mLeftShooterPID.setFF(Constants.Shooter.kShooterFF);
    mLeftShooterPID.setOutputRange(Constants.Shooter.kShooterMinOutput, Constants.Shooter.kShooterMaxOutput);

    mRightShooterPID = mRightShooterMotor.getPIDController();
    mRightShooterPID.setP(Constants.Shooter.kShooterP);
    mRightShooterPID.setI(Constants.Shooter.kShooterI);
    mRightShooterPID.setD(Constants.Shooter.kShooterD);
    mRightShooterPID.setFF(Constants.Shooter.kShooterFF);
    mRightShooterPID.setOutputRange(Constants.Shooter.kShooterMinOutput, Constants.Shooter.kShooterMaxOutput);

    mLeftShooterEncoder = mLeftShooterMotor.getEncoder();
    mRightShooterEncoder = mRightShooterMotor.getEncoder();

    mLeftShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    mRightShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    mLeftShooterMotor.setInverted(true);
    mRightShooterMotor.setInverted(false);
  }

  private static class PeriodicIO {
    double shooter_rpm = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  /*
  @Override
  public void periodic() {
  }
  */

  @Override
  public void writePeriodicOutputs() {
    double limitedSpeed = mSpeedLimiter.calculate(mPeriodicIO.shooter_rpm);
    mLeftShooterPID.setReference(limitedSpeed, ControlType.kVelocity);
    mRightShooterPID.setReference(limitedSpeed, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    stopShooter();
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Speed (RPM):", mPeriodicIO.shooter_rpm);
    SmartDashboard.putNumber("Left speed:", mLeftShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Right speed:", mRightShooterEncoder.getVelocity());
  }

  /*
  @Override
  public void reset() {
  }
  */

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void setSpeed(double rpm) {
    mPeriodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() {
    mPeriodicIO.shooter_rpm = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
