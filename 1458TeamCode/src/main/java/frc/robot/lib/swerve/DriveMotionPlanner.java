package frc.robot.lib.swerve;

//dc.10.21.2024, rewrite citrus code using wpilib Trajectory classes
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.lib.control.Lookahead;
import frc.robot.lib.util.Util;

public class DriveMotionPlanner {

    TrajectoryIterator mCurrentTrajectory;

    double mDt = 0.0;   //delta of time
    boolean mIsReversed = false;
	double mLastTime = Double.POSITIVE_INFINITY;
/* 	public TimedState<Pose2dWithMotion> mLastSetpoint = null;*/ //dc.10.21.2024, mLastSetpoint seems only to appear on left and zero right-side reference
	public Trajectory.State mSetpoint = new Trajectory.State(0.,0.,0.,new Pose2d(0.,0.,new Rotation2d(0)),0.);
	Pose2d mError = new Pose2d(0, 0, new Rotation2d(0));
/*  ErrorTracker mErrorTracker = new ErrorTracker(15 * 100);   */ //dc.10.21.2024, mErrorTracker seems only to appear on left and its getty function has zero reference. 
	Translation2d mTranslationalError = new Translation2d(0, 0);
	Rotation2d mPrevHeadingError = new Rotation2d(0);
	Pose2d mCurrentState = new Pose2d(0, 0, new Rotation2d(0));
	
	double mCurrentTrajectoryLength = 0.0;
	double mTotalTime = Double.POSITIVE_INFINITY;
	double mStartTime = Double.POSITIVE_INFINITY;
	ChassisSpeeds mOutput = new ChassisSpeeds();

	Lookahead mSpeedLookahead = null;



    //constructor code 
    public DriveMotionPlanner() {}

    //set trajectory to traverse
    public void setTrajectory(final TrajectoryIterator trajectory) {
        mCurrentTrajectory = trajectory;
        //TODO: actual setTrajectory() to be implemented 
    }

	public void reset() {
//		mErrorTracker.reset();
		mTranslationalError = new Translation2d();
		mPrevHeadingError = new Rotation2d();
//		mLastSetpoint = null;
		mOutput = new ChassisSpeeds();
		mLastTime = Double.POSITIVE_INFINITY;
	}


    //follower type
	public enum FollowerType {
		FEEDFORWARD_ONLY,
		PID,
		PURE_PURSUIT,
		RAMSETE
	}

	FollowerType mFollowerType = FollowerType.PURE_PURSUIT;

	public void setFollowerType(FollowerType type) {
		mFollowerType = type;
	}

    // update chassis speeds at the specified timestamp based on current Pose2d and Velocity
	public ChassisSpeeds update(double timestamp, Pose2d current_state, Translation2d current_velocity) {
		if (mCurrentTrajectory == null) return null;

		if (!Double.isFinite(mLastTime)) mLastTime = timestamp;
		mDt = timestamp - mLastTime;
		mLastTime = timestamp;
		Trajectory.State sample_point;  //dc. replace citrus TrajectorySamplePoint with wpilib trajectory.state
		mCurrentState = current_state;
		Twist2d pid_error;	//twist2d between actual and desired states. 


		if (!isDone()) {
			// Compute error in robot frame
			mPrevHeadingError = mError.getRotation();
			mError = calculateError(current_state, mSetpoint.poseMeters);
			pid_error = current_state.log(mSetpoint.poseMeters);//* calculate the Twist2d delta/error between actualState and the desired state.  citrus original code is //Pose2d.log(mError);			
//dc.10.21.2024			mErrorTracker.addObservation(mError);
			if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
				sample_point = mCurrentTrajectory.advance(mDt);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;

				final double velocity_m = mSetpoint.velocityMetersPerSecond;
				// Field relative
				/* 
				 *dc.10.22.2024 replace with code based on wpilib Trajectory.State
				var course = mSetpoint.state().getCourse();
				Rotation2d motion_direction = course.isPresent() ? course.get() : Rotation2d.identity();
				// Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
				motion_direction = current_state.getRotation().inverse().rotateBy(motion_direction);
				*/
				Rotation2d motion_direction = mSetpoint.poseMeters.getRotation(); // Get the planned course of motion from the trajectory (the robot's desired rotation)
				motion_direction = current_state.getRotation().unaryMinus().rotateBy(motion_direction);

				mOutput = new ChassisSpeeds(
						motion_direction.getCos() * velocity_m,
						motion_direction.getSin() * velocity_m,
						// Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
						velocity_m * mSetpoint.curvatureRadPerMeter); //state().getHeadingRate());
			} else if (mFollowerType == FollowerType.RAMSETE) {
				sample_point = mCurrentTrajectory.advance(mDt);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;
				// mOutput = updateRamsete(sample_point.state(), current_state, current_velocity);
			} else if (mFollowerType == FollowerType.PID) {
				sample_point = mCurrentTrajectory.advance(mDt);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;

				final double velocity_m = mSetpoint.velocityMetersPerSecond;
				// Field relative
				/* 
				 *dc.10.22.2024 replace with code based on wpilib Trajectory.State
				var course = mSetpoint.state().getCourse();
				Rotation2d motion_direction = course.isPresent() ? course.get() : Rotation2d.identity();
				// Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
				motion_direction = current_state.getRotation().inverse().rotateBy(motion_direction);
				*/
				Rotation2d motion_direction = mSetpoint.poseMeters.getRotation(); // Get the planned course of motion from the trajectory (the robot's desired rotation)
				motion_direction = current_state.getRotation().unaryMinus().rotateBy(motion_direction);

				var chassis_speeds = new ChassisSpeeds(
						motion_direction.getCos() * velocity_m,
						motion_direction.getSin() * velocity_m,
						// Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
						velocity_m * mSetpoint.curvatureRadPerMeter);
				// PID is in robot frame
				mOutput = updatePIDChassis(chassis_speeds, pid_error);
			} else if (mFollowerType == FollowerType.PURE_PURSUIT) {
				double searchStepSize = 1.0;
				double previewQuantity = 0.0;
				double searchDirection = 1.0;
				double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
				double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
				searchDirection = Math.signum(reverseDistance - forwardDistance);
				while (searchStepSize > 0.001) {
					SmartDashboard.putNumber("PurePursuit/PreviewDist", distance(current_state, previewQuantity));
					if (Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.0003937)) break;
					while (
					/* next point is closer than current point */ distance(
									current_state, previewQuantity + searchStepSize * searchDirection)
							< distance(current_state, previewQuantity)) {
						/* move to next point */
						previewQuantity += searchStepSize * searchDirection;
					}
					searchStepSize /= 10.0;
					searchDirection *= -1;
				}
				SmartDashboard.putNumber("PurePursuit/PreviewQtd", previewQuantity);
				sample_point = mCurrentTrajectory.advance(previewQuantity);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;
				mOutput = updatePurePursuit(current_state, 0.0);
			}
		} else {
			if (mCurrentTrajectory.getLastPoint().velocityMetersPerSecond == 0.0) {
				mOutput = new ChassisSpeeds();
			}
		}

		return mOutput;
	}

    // check if we complete the current trajectory
    public boolean isDone() {
		return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
	}

	//dc. add Twist2d pid_error as input parameter to remove dependency on class property in original citrus code
	protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds, Twist2d pid_error) {
		// dc.10.22.2024, TODO: tune the "K" constants of PID algo
		// Feedback on longitudinal error (distance).
		final double kPathk = 2.4; // 2.4;/* * Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)*/;//0.15;
		final double kPathKTheta = 3.0;
		chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond + kPathk * pid_error.dx;
		chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond + kPathk * pid_error.dy;
		chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + kPathKTheta * pid_error.dtheta;
		return chassisSpeeds;
	}

	//dc.10.22.2024 TODO: updatePurePursuit() to be ported from Citrus code
	protected ChassisSpeeds updatePurePursuit(Pose2d current_state, double feedforwardOmegaRadiansPerSecond) {		
		return null;
	}

	//getty functions to access key properties 
	public synchronized Translation2d getTranslationalError() {
		return new Translation2d(
				mError.getTranslation().getX(), mError.getTranslation().getY());
	}
	public synchronized Rotation2d getHeadingError() {
		return mError.getRotation();
	}



	// dc.10.21.2024 calculate the error between robot's actual state and desired state
	// TODO: revisit to check the math. citrus original code is : current_state.inverse().transformBy(mSetpoint.state().getPose());
	public Pose2d calculateError (Pose2d actualState, Pose2d desiredState){
		// Calculate the error in translation (position)
		Translation2d positionError = desiredState.getTranslation().minus(actualState.getTranslation());

		// Calculate the error in rotation (orientation)
		Rotation2d rotationError = desiredState.getRotation().minus(actualState.getRotation());

		// Create a new Pose2d object to represent the error
		return new Pose2d(positionError, rotationError);
	}

	//dc.10.24.202 
	private double distance(Pose2d current_state, double additional_progress) {
		Trajectory.State previewState = mCurrentTrajectory.preview(additional_progress);
		//TODO: revisit to check the math. citrus original code is : return Pose2d.log(previewState.inverse().transformBy(current_state)).norm());
		Twist2d logErr = previewState.poseMeters.log(calculateError(previewState.poseMeters, current_state));
		// get the norm of twist2d object
		if (logErr.dy == 0.0)
			return Math.abs(logErr.dx);
		return Math.hypot(logErr.dx, logErr.dy);
	}	
}
