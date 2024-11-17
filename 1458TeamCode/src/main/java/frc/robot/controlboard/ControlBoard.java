package frc.robot.controlboard;

//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.robot.lib.util.Util;
import frc.robot.lib.math.Translation2d;
import frc.robot.lib.math.Rotation2d;

public class ControlBoard {
	private final double kSwerveDeadband = Constants.stickDeadband;

	private static ControlBoard mInstance = null;

	public static ControlBoard getInstance() {
		if (mInstance == null) {
			mInstance = new ControlBoard();
		}

		return mInstance;
	}

	public final CustomXboxController driver;

	public ControlBoard() {
		driver = new CustomXboxController(0);
	}

    public void update() {
		driver.update();
	}

	public Translation2d getSwerveTranslation() {
		double forwardAxis = driver.getRawAxis(Axis.kLeftY.value);
		double strafeAxis = driver.getRawAxis(Axis.kLeftX.value);

		SmartDashboard.putNumber("Raw Y", forwardAxis);
		SmartDashboard.putNumber("Raw X", strafeAxis);

		forwardAxis = Constants.SwerveConstants.invertYAxis ? forwardAxis : -forwardAxis;
		strafeAxis = Constants.SwerveConstants.invertXAxis ? strafeAxis : -strafeAxis;

		Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

		if (Math.abs(tAxes.norm()) < kSwerveDeadband) {
			return new Translation2d();
		} else {
			Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
			Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, kSwerveDeadband);

			double scaled_x = Util.scaledDeadband(forwardAxis, 1.0, Math.abs(deadband_vector.x()));
			double scaled_y = Util.scaledDeadband(strafeAxis, 1.0, Math.abs(deadband_vector.y()));
			return new Translation2d(scaled_x, scaled_y)
					.scale(Drive.getInstance().getKinematicLimits().kMaxDriveVelocity);
		}
	}

	public double getSwerveRotation() {
		double rotAxis = driver.getRightX() * 0.80;
		rotAxis = Constants.SwerveConstants.invertRAxis ? rotAxis : -rotAxis;

		if (Math.abs(rotAxis) < kSwerveDeadband) {
			return 0.0;
		} else {
			return Drive.getInstance().getKinematicLimits().kMaxAngularVelocity
					* (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband))
					/ (1 - kSwerveDeadband);
		}
	}
}
