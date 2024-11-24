package frc.robot.lib.trajectory;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.DummySubsystem;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.io.IOException;

//dc.10.21.2024, this class is going to load all paths pre-defined via PathWeaver tool

public class TrajectoryGenerator {
    
    //the trajectories used in auto mode
    private TrajectorySet mTrajectorySet = null;

    public class TrajectorySet {
        
		public Trajectory testTrajectory = loadTrajectory("paths/output/pathweaver1.wpilib.json"); //the parent folder NEED to be "./src/main/deploy/"
        /* dc.10.21.2024, additional trajectory can be added similar to the TestTrajectory */

        private Trajectory loadTrajectory (String sJsonFile){
            try{
                // Get the path to the deployed JSON file
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(sJsonFile);                
                // Load the trajectory
                Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                System.out.println("Trajectory loaded successfully! =" + trajectoryPath.toString());
                return trajectory;
            } catch(IOException err){
                System.out.println("Trajectory loaded failed! =" + sJsonFile);
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
