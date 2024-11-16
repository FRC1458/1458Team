package frc.robot;

import org.opencv.core.TickMeter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.limelight.LimeLight;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    public final Drive s_Swerve = new Drive();
    private final LimeLight mLimeLight = new LimeLight();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        TeleopSwerve teleopSwerveCommand = new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()
        );
        s_Swerve.setDefaultCommand(teleopSwerveCommand);

        SmartDashboard.putData("TeleopSwerveCmd", teleopSwerveCommand);

        CommandScheduler.getInstance().registerSubsystem(mLimeLight);
        
        // Configure the button bindings
        configureButtonBindings();
        // Call the method to display Limelight data on SmartDashboard
        displayLimeLightData();
        configureShuffleboard();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new Auto(s_Swerve);
    }

    // Method to display Limelight data on SmartDashboard
    public void displayLimeLightData() {
        SmartDashboard.putBoolean("Limelight Has Target", mLimeLight.hasTarget());
        SmartDashboard.putNumber("Limelight X", mLimeLight.getX());
        SmartDashboard.putNumber("Limelight Y", mLimeLight.getY());
        SmartDashboard.putNumber("Limelight Area", mLimeLight.getArea());
    }

    // Call this method periodically (e.g., in a periodic method in your main robot class)
    public void updateLimeLightData() {
        SmartDashboard.putBoolean("Limelight Has Target", mLimeLight.hasTarget());
        SmartDashboard.putNumber("Limelight X", mLimeLight.getX());
        SmartDashboard.putNumber("Limelight Y", mLimeLight.getY());
        SmartDashboard.putNumber("Limelight Area", mLimeLight.getArea());
        SmartDashboard.putNumber("Red TIe R", mLimeLight.getArea());
        Shuffleboard.update();
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("LimeLight Data");
        tab.addBoolean("Limelight Has Target", mLimeLight::hasTarget);
        tab.addNumber("Limelight X", mLimeLight::getX);
        tab.addNumber("Limelight Y", mLimeLight::getY);
        tab.addNumber("Limelight Area", mLimeLight::getArea);
        tab.addNumber("Red TIe R", mLimeLight::getArea);

        ShuffleboardTab tab1 = Shuffleboard.getTab("Module");
        tab1.addNumber("Module", mLimeLight::getX);
    }
    
}