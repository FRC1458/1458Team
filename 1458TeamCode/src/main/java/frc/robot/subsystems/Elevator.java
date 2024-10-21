// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  // private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);
  // private static final TalonFX elevatorGearbox = new TalonFX(Constants.Elevator.deviceID);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController elevatorMotorController =
      new ProfiledPIDController(
          Constants.Elevator.kElevatorKP,
          Constants.Elevator.kElevatorKI,
          Constants.Elevator.kElevatorKD,
          new TrapezoidProfile.Constraints(2.45, 2.45));

  ElevatorFeedforward feedForward =
      new ElevatorFeedforward(
          Constants.Elevator.kElevatorKS,
          Constants.Elevator.kElevatorKG,
          Constants.Elevator.kElevatorKV,
          Constants.Elevator.kElevatorKA);

  private final Encoder encoder =
      new Encoder(Constants.Elevator.kEncoderAChannel, Constants.Elevator.kEncoderBChannel);

  // private final PWMSparkMax elevatorMotor = new PWMSparkMax(Constants.Elevator.kMotorPort);
  // private final PWMSim elevatorMotorSim = new PWMSim(elevatorMotor);
  private static final TalonFX elevatorMotor = new TalonFX(Constants.Elevator.deviceID);
  private final TalonFXSimState elevatorMotorSim = elevatorMotor.getSimState();

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          // m_elevatorGearbox,
          DCMotor.getVex775Pro(4),
          Constants.Elevator.kElevatorGearing,
          Constants.Elevator.kCarriageMass,
          Constants.Elevator.kElevatorDrumRadius,
          Constants.Elevator.kMinElevatorHeightMeters,
          Constants.Elevator.kMaxElevatorHeightMeters,
          true,
          0);
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d mech = new Mechanism2d(0.2, 0.5);
  private final MechanismRoot2d elevator = mech.getRoot("Elevator", 0.25, 0);
  // private final MechanismLigament2d elevatorViz =
  //     elevator.append(
  //         new MechanismLigament2d(
  //           "Shaft", 20, 90, 5.0, new Color8Bit(Color.kYellow)
  //       )
  //     );
  private final MechanismLigament2d elevatorViz =
      elevator.append(
          new MechanismLigament2d(
            "Shaft", 0.4, 90, 5.0, new Color8Bit(Color.kYellow)
        )
      );

  /** Subsystem constructor. */
  public Elevator() {
    encoder.setDistancePerPulse(Constants.Elevator.kElevatorEncoderDistPerPulse);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator
    SmartDashboard.putData("Elevator", mech);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Target Position", elevatorMotorController.getGoal().position);
    SmartDashboard.putNumber("Elevator Actual Position", elevatorMotor.getPosition().getValue());
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    // m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());
    // elevatorSim.setInput(elevatorMotorSim.getSpeed());
    elevatorSim.setInput(elevatorMotorSim.getMotorVoltage());

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(TimedRobot.kDefaultPeriod);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // encoderSim.setDistance(elevatorSim.getPositionMeters());
    elevatorMotorSim.setRawRotorPosition(elevatorSim.getPositionMeters());
    elevatorMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // elevatorViz.setLength(encoder.getDistance());
    // elevatorViz.setLength(elevatorSim.getPositionMeters());
    // elevatorViz.setLength(
    //   elevatorViz.getLength()
    //     + Math.toDegrees(elevatorSim.getVelocityMetersPerSecond())
    //       * TimedRobot.kDefaultPeriod);
    elevatorViz.setLength(elevatorViz.getLength()
      + elevatorSim.getVelocityMetersPerSecond() * TimedRobot.kDefaultPeriod);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    elevatorMotorController.setGoal(goal);

    // With the setpoint value we run PID control like normal
    double pidOutput = elevatorMotorController.calculate(encoder.getDistance());
    double feedforwardOutput =
        feedForward.calculate(MetersPerSecond.of(elevatorMotorController.getSetpoint().velocity).magnitude());
    elevatorMotor.setVoltage(pidOutput + feedforwardOutput);
  }

  // /** Stop the control loop and motor output. */
  // public void stop() {
  //   elevatorMotorController.setGoal(0.0);
  //   elevatorMotor.set(0.0);
  // }

  // /** Update telemetry, including the mechanism visualization. */
  // public void updateTelemetry() {
  //   // Update elevator visualization with position
  //   elevatorViz.setLength(encoder.getDistance());
  // }

  // @Override
  // public void close() {
  //   m_encoder.close();
  //   m_motor.close();
  //   m_mech2d.close();
  // }
}