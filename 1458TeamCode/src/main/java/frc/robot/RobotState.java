package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

//dc.10.25.2024 TODO: a dummy RobotState class to pass compilation, placeholder for actual implementation
//priority level 2, for presentation purpose
public class RobotState {
	private static RobotState mInstance;

	public static RobotState getInstance() {
		if (mInstance == null) {
			mInstance = new RobotState();
		}
		return mInstance;
	}

	/**
	 * Adds new odometry pose update.
	 *
	 * @param now                Timestamp of observation.
	 * @param odometry_pose      Reported pose from odometry.
	 * @param measured_velocity  Measured field-relative velocity.
	 * @param predicted_velocity Predicted field-relative velocity (usually swerve
	 *                           setpoint).
	 */
	public synchronized void addOdometryUpdate(double now, Pose2d odometry_pose, Twist2d measured_velocity, Twist2d predicted_velocity) {
        //TODO: actual addOdometryUpdate() to be implemented 
    }

	/**
	 * Gets initial odometry error. Odometry initializes to the origin, while the
	 * robot starts at an unknown position on the field.
	 *
	 * @return Initial odometry error translation.
	 */
	public synchronized Translation2d getInitialFieldToOdom() {
        //TODO: actual getInitialFieldToOdom() to be implemented 
        return null;
	}

	/**
	 * @return Latest field relative robot pose.
	 */
	public synchronized Pose2d getLatestFieldToVehicle() {
        //TODO: actual getLatestFieldToVehicle() to be implemented 
        return null;
	}

	/**
	 * Gets field relative robot pose from history. Linearly interpolates between
	 * gaps.
	 *
	 * @param timestamp Timestamp to look up.
	 * @return Field relative robot pose at timestamp.
	 */
	public synchronized Pose2d getFieldToVehicle(double timestamp) {
        //TODO: actual getFieldToVehicle() to be implemented 
        return null;
	}

	/**
	 * Gets interpolated robot pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookahead_time Scalar for predicted velocity.
	 * @return Predcited robot pose at lookahead time.
	 */
	public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        //TODO: actual getPredictedFieldToVehicle() to be implemented 
        return null;
	}

	/**
	 * @return Latest odometry pose.
	 * TODO: find replacement of InterpolatingDouble, or port team254 code
	public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestOdomToVehicle() {
        //TODO: actual getLatestOdomToVehicle() to be implemented 
        return null;
	}
	 */

	/**
	 * Gets odometry pose from history. Linearly interpolates between gaps.
	 *
	 * @param timestamp Timestamp to loop up.
	 * @return Odometry relative robot pose at timestamp.
	 */
	public synchronized Pose2d getOdomToVehicle(double timestamp) {
        //TODO: actual getOdomToVehicle() to be implemented 
        return null;
	}

	/**
	 * Gets interpolated odometry pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookahead_time Scalar for predicted velocity.
	 * @return Predcited odometry pose at lookahead time.
	 */
	public synchronized Pose2d getPredictedOdomToVehicle(double lookahead_time) {
        //TODO: actual getPredictedOdomToVehicle() to be implemented 
        return null;
	}

	/**
	 * @return Latest odometry error translation.
	 */
	public synchronized Translation2d getLatestFieldToOdom() {
        //TODO: actual getLatestFieldToOdom() to be implemented 
        return null;
	}

	/**
	 * Gets odometry error translation at timestamp. Linearly interpolates between gaps.
	 * @param timestamp Timestamp to look up.
	 * @return Odometry error at timestamp.
	 */
	public synchronized Translation2d getFieldToOdom(double timestamp) {
        //TODO: actual getFieldToOdom() to be implemented 
        return null;
	}

	/**
	 * @return Predicted robot velocity from last odometry update.
	 */
	public synchronized Twist2d getPredictedVelocity() {
        //TODO: actual getPredictedVelocity() to be implemented 
        return null;
	}

	/**
	 * @return Measured robot velocity from last odometry update.
	 */
	public synchronized Twist2d getMeasuredVelocity() {
        //TODO: actual getMeasuredVelocity() to be implemented 
        return null;
	}

	/**
	 * @return Measured robot velocity smoothed using a moving average filter.
	 */
	public synchronized Twist2d getSmoothedVelocity() {
        //TODO: actual getSmoothedVelocity() to be implemented 
        return null;
	}

	/**
	 * @return Gets if estimator has recieved a vision update.
	 */
	public synchronized boolean getHasRecievedVisionUpdate() {
        //TODO: actual getHasRecievedVisionUpdate() to be implemented 
        return false;
	}    
}
