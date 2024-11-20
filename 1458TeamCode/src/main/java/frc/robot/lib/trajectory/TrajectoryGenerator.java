package frc.robot.lib.trajectory;

import edu.wpi.first.math.trajectory.*;
import frc.robot.subsystems.DummySubsystem;

import java.nio.file.Paths;
import java.io.IOException;

//dc.10.21.2024, this class is going to load all paths pre-defined via PathWeaver tool

public class TrajectoryGenerator {
    
    //the trajectories used in auto mode
    private TrajectorySet mTrajectorySet = null;

    public class TrajectorySet {
        
		public Trajectory testTrajectory = loadTrajectory("./PathWeaver/output/Unnamed_0.wpilib.json"); //TODO: update with right path of the test json file here. 
        /* dc.10.21.2024, additional trajectory can be added similar to the TestTrajectory */

        private Trajectory loadTrajectory (String sJsonFile){
            try{
                return TrajectoryUtil.fromPathweaverJson( Paths.get(sJsonFile));
            } catch(IOException err){
                return null;
            }
        }
    }

    // instanciation code 
    private static TrajectoryGenerator mInstance;
	public static TrajectoryGenerator getInstance() {
		if (mInstance == null) {
			mInstance = new TrajectoryGenerator();
		}
		return mInstance;
	}

    //constructor code
    public TrajectoryGenerator (){
        //dc.to be implemented
        //TODO: finish trajectory generator constructor
    }

    //actually create the trajectory object
    public void generateTrajectories (){
        if (mTrajectorySet == null) {
			System.out.println("Generating trajectories...");
			mTrajectorySet = new TrajectorySet();
			System.out.println("Finished trajectory generation");
		}
    }    

    //access to the trajectory set 
	public TrajectorySet getTrajectorySet() {
		return mTrajectorySet;
	}
}
