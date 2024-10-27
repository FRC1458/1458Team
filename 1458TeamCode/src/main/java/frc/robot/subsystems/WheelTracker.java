package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.swerve.SwerveModule;

//dc.10.21.2024 TODO: a dummy WheelTracker class to pass compilation, placeholder for actual implementation
public class WheelTracker {
    //TODO: WheelTracker entire class needs to be implemented
    public WheelTracker(SwerveModule[] modules) {}
	public synchronized Pose2d getRobotPose() {	return null;}
    public void stop() {}
    public void start() {}
    public Translation2d getMeasuredVelocity() {return null;}
    public void resetPose(Pose2d wantedPose){}


    
}
