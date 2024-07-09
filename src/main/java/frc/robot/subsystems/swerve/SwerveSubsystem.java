package frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.function.Supplier;

import com.google.common.collect.Streams;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Module.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {

    // Drivebase constants
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16);
    public static final double MAX_LINEAR_ACCELERATION = 8.0;
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(21.75);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(21.25);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final double MAX_ANGULAR_ACCELERATION = MAX_LINEAR_ACCELERATION / DRIVE_BASE_RADIUS;
    public static final double MAX_AUTOAIM_SPEED = MAX_LINEAR_SPEED / 4;
    // Hardware constants
    public static final int PIGEON_ID = 0;

    public static final ModuleConstants frontLeft =
      new ModuleConstants("Front Left", 0, 1, 0, Rotation2d.fromRotations(0.377930));
  public static final ModuleConstants frontRight =
      new ModuleConstants("Front Right", 2, 3, 1, Rotation2d.fromRotations(-0.071289));
  public static final ModuleConstants backLeft =
      new ModuleConstants("Back Left", 4, 5, 2, Rotation2d.fromRotations(0.550781));
  public static final ModuleConstants backRight =
      new ModuleConstants("Back Right", 6, 7, 3, Rotation2d.fromRotations(-0.481689));

    // private final GyroIO gyroIO;
    private final GyroIO gyroIO;
    private final Module[] modules; // FL, FR, BL, BR
    
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Pose2d pose = new Pose2d();

    public SwerveSubsystem(GyroIO gyroIO, ModuleIO[] moduleIOs) {
        this.gyroIO = gyroIO;
        modules = new Module[moduleIOs.length];

        for (int i = 0; i < moduleIOs.length; i++) {
            modules[i] = new Module(moduleIOs[i]);
        }
        PhoenixOdometryThread.getInstance().start();
    }

    /**
     * Constructs an array of swerve module ios corresponding to the real robot.
     *
     * @return The array of swerve module ios.
     */
    public static ModuleIO[] createTalonFXModules() {
        return new ModuleIO[] {
                new ModuleIOReal(frontLeft),
                new ModuleIOReal(frontRight),
                new ModuleIOReal(backLeft),
                new ModuleIOReal(backRight)
        };
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
        new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
        new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
        new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
        new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
        };
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public Command runVelocityCmd(Supplier<ChassisSpeeds> speeds) {
        return this.run(() -> runVelocity(speeds.get()));
    }

    /** Stops the drive. */
    public Command stopCmd() {
        return runVelocityCmd(ChassisSpeeds::new);
    }

    public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
        return this.runVelocityCmd(
                () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), pose.getRotation()));
    }

    public Command runVelocityTeleopFieldRelative(Supplier<ChassisSpeeds> speeds) {
        return this.runVelocityCmd(
                () -> ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds.get(),
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                ? pose.getRotation()
                                : pose.getRotation().minus(Rotation2d.fromDegrees(180))));
    }

    private void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);
    
        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates =
            Streams.zip(
                    Arrays.stream(modules), Arrays.stream(setpointStates), (m, s) -> m.runSetpoint(s))
                .toArray(SwerveModuleState[]::new);
      }

      public ChassisSpeeds getVelocity() {
        var speeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                kinematics.toChassisSpeeds(
                    Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new)),
                pose.getRotation());
        return new ChassisSpeeds(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
      }

    public Command runVoltageTeleopFieldRelative(Supplier<ChassisSpeeds> speeds) {
        return this.run(
                () -> {
                    var allianceSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds.get(),
                            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                    ? pose.getRotation()
                                    : pose.getRotation().minus(Rotation2d.fromDegrees(180)));
                    // Calculate module setpoints
                    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(allianceSpeeds, 0.02);
                    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
                    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

                    final boolean focEnable = Math.sqrt(
                            Math.pow(this.getVelocity().vxMetersPerSecond, 2)
                                    + Math.pow(this.getVelocity().vyMetersPerSecond, 2)) < MAX_LINEAR_SPEED * 0.9;

                    // Send setpoints to modules
                    SwerveModuleState[] optimizedSetpointStates = Streams.zip(
                            Arrays.stream(modules),
                            Arrays.stream(setpointStates),
                            (m, s) -> m.runVoltageSetpoint(
                                    new SwerveModuleState(
                                            s.speedMetersPerSecond * 12.0 / MAX_LINEAR_SPEED, s.angle),
                                    focEnable))
                            .toArray(SwerveModuleState[]::new);
                });
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public Command stopWithXCmd() {
        return this.run(
                () -> {
                    Rotation2d[] headings = new Rotation2d[4];
                    for (int i = 0; i < modules.length; i++) {
                        headings[i] = getModuleTranslations()[i].getAngle();
                    }
                    kinematics.resetHeadings(headings);
                    for (int i = 0; i < modules.length; i++) {
                        modules[i].runSetpoint(new SwerveModuleState(0.0, headings[i]));
                    }
                });
    }

}
