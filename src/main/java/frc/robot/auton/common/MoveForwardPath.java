package frc.robot.auton.common;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.*;
import frc.robot.subsystems.*;

// GP = game piece
// Can be used to place one cube or one cone and either starting position one or two
public class MoveForwardPath extends SequentialCommandGroup {

    private double distance;
	
	public MoveForwardPath(SwerveDrivetrain drivetrain, RobotContainer container, double distance) {

        this.distance = distance;
		
		addCommands(
			new DrivetrainSwerveRelative(drivetrain, container, createMoveForwardTrajectory(container))
        ); 
  
    }
    
    public Trajectory createMoveForwardTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = 

		return trajectory;
	}

}