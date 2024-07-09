// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.CommandXboxControllerSubsystem;

public class Robot extends TimedRobot {
  public static enum RobotMode {
    SIM,
    REPLAY,
    REAL
  }

  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private final CommandXboxControllerSubsystem controller = new CommandXboxControllerSubsystem(0);
  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.REPLAY;

  private final SwerveSubsystem swerve =
      new SwerveSubsystem(
          new GyroIOPigeon2(),
          SwerveSubsystem.createTalonFXModules()
      );

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();

    swerve.setDefaultCommand(
        swerve.runVoltageTeleopFieldRelative(
            () ->
                new ChassisSpeeds(
                    -teleopAxisAdjustment(controller.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                    -teleopAxisAdjustment(controller.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                    -teleopAxisAdjustment(controller.getRightX())
                        * SwerveSubsystem.MAX_ANGULAR_SPEED)));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = Commands.none();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /** Modifies the given joystick axis value to make teleop driving smoother. */
  private static double teleopAxisAdjustment(double x) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(x, 2)) * Math.signum(x), 0.02);
  }
}
