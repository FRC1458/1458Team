package frc.robot.lib.trajectory;

import java.util.List;

import edu.wpi.first.math.trajectory.*;

//dc.10.21.2024, rewrite the TrajectoryIterator class based on wpilib Trajectory package, main functions as following
//
//2. rewrite the advance () method which is used often by its callers
//
public class TrajectoryIterator {
    protected double progress_ = 0.0;
    protected Trajectory.State current_sample_;     //corresponding to TrajectorySamplePoint in citrus code
    protected Trajectory mCurrentTrajectory=null;

    //construtor code
    public TrajectoryIterator (Trajectory curTrajectory){
        mCurrentTrajectory=curTrajectory;
        current_sample_ = curTrajectory.getStates().get(0);
        //progress_ = view_.first_interpolant();        //dc. is it not zero??
    }

    //advance by additional time on the trajectory
    public Trajectory.State advance (double additional_progress){
        progress_ = Math.max(0.0, Math.min(mCurrentTrajectory.getTotalTimeSeconds(), progress_ + additional_progress));
        current_sample_ = mCurrentTrajectory.sample(progress_);
        return current_sample_;
    }

    //preview the trajectory
    public Trajectory.State preview (double additional_progress){
        final double progress = Math.max(0.0, Math.min(mCurrentTrajectory.getTotalTimeSeconds(), progress_ + additional_progress));
        return mCurrentTrajectory.sample(progress);
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return progress_;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, mCurrentTrajectory.getTotalTimeSeconds() - progress_);
    }

    public Trajectory.State getLastPoint(){
        List<Trajectory.State> stateList = mCurrentTrajectory.getStates();
        return stateList.get(stateList.size()-1);
    }

    //access to the current trajectory properties
    public Trajectory trajectory(){ return mCurrentTrajectory;}

    //get the current state
    public Trajectory.State getState(){
        return current_sample_;
    }

}
