package frc.robot.autos.modes;

import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;
import frc.robot.autos.actions.SwerveTrajectoryAction;
import frc.robot.lib.trajectory.TrajectoryGenerator;
import frc.robot.lib.trajectory.TrajectoryGenerator.TrajectorySet;

public class TestPathMode extends AutoModeBase {
	private TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();
	private TrajectorySet trajectorySet = trajectoryGenerator.getTrajectorySet();
	public TestPathMode() {}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		System.out.println("Running test mode auto!");
		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectory, true));
		System.out.println("Finished auto!");
	}
	// spotless:on
}
