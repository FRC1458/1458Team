package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.math.Conversions;
import frc.robot.Constants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private double wheelCircumference;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward;

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(SwerveModuleConfig swerveModuleConfig){
        moduleNumber = swerveModuleConfig.moduleNumber;
        angleOffset = swerveModuleConfig.angleMotor.angleOffset;
        wheelCircumference = swerveModuleConfig.wheelCircumference;

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
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, this.wheelCircumference);
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
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), this.wheelCircumference),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), this.wheelCircumference),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public double getAngleMotorPosition(){
        // return Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()).getDegrees();
        return mAngleMotor.getPosition().getValue();
    }
}