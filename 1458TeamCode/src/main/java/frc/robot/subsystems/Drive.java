package frc.robot.subsystems;


import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public Module[] mSwerveMods;
    public Pigeon2 gyro;

    private Trajectory trajectory;
    private final Field2d field = new Field2d();
    private Pose2d simPose = new Pose2d();

    // // Simulation
    // private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    // private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    // private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    private final Pigeon2SimState gyroSim;

    public Drive() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "CV");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new Module[]{
                new Module(0, Constants.Swerve.FrontLeftMod.constants),
                new Module(1, Constants.Swerve.FrontRightMod.constants),
                new Module(2, Constants.Swerve.BackLeftMod.constants),
                new Module(3, Constants.Swerve.BackRightMod.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        // Simulation
        // Create and push Field2d to SmartDashboard.
        SmartDashboard.putData("Field", field);

        // Create the trajectory to follow in autonomous.
        trajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0))
            );

        // Push the trajectory to Field2d.
        field.getObject("traj").setTrajectory(trajectory);

        gyroSim = gyro.getSimState();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                getHeading()
                        )
                                : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (Module mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (Module mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (Module mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (Module mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (Module mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = getModuleStates();
        for (int i = 0; i < mSwerveMods.length; i++) {
            double goalDegrees = states[i].angle.getDegrees();
            SmartDashboard.putNumber("Module " + mSwerveMods[i].moduleNumber + " Goal Angle", goalDegrees);
        }
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (Module mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", (mod.getCANcoder().getDegrees()+180)%360);

//            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", ((mod.mAngleMotor.getPosition().getValue()%22.0)*360/22-180));
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", ((mod.mAngleMotor.getPosition().getValue() * 360) % 360 + 360) % 360); // This is super specific, don't break this pls
//            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        field.setRobotPose(swerveOdometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Sim jointly as one system
        for (Module mod : mSwerveMods) {
            mod.updateSimPeriodic();

            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", (mod.getCANcoder().getDegrees()+180)%360);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", ((mod.mAngleMotor.getPosition().getValue() * 360) % 360 + 360) % 360); // This is super specific, don't break this pls
        }

        // final ChassisSpeeds chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        // simPose = simPose.transformBy((
        //     new Transform2d(
        //         new Translation2d(
        //             chassisSpeeds.vxMetersPerSecond * TimedRobot.kDefaultPeriod * 100000,
        //             chassisSpeeds.vyMetersPerSecond * TimedRobot.kDefaultPeriod * 100000),
        //         // new Translation2d(
        //         //     20,
        //         //     20),
        //         new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * TimedRobot.kDefaultPeriod)
        //     )
        // ));
        // field.setRobotPose(simPose);

        swerveOdometry.update(getGyroYaw(), getModulePositions());
        field.setRobotPose(swerveOdometry.getPoseMeters());
    }
}