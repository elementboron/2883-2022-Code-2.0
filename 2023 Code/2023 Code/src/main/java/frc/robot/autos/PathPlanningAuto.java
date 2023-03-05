package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.StopRobotAutonomous;
import frc.robot.subsystems.Swerve;

public class PathPlanningAuto extends SequentialCommandGroup {
    public PathPlanningAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
    
        //PathPlannerTrajectory traj = PathPlanner.loadPath("DriveStraighPath.path", new PathConstraints(3, 1));
        PathPlannerTrajectory traj = PathPlanner.generatePath(
    new PathConstraints(3, 1), 
    new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading
    new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(45)) // position, heading
    );


    



        addCommands(
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new FollowTrajectory(s_Swerve, traj, true)
            //new StopRobotAutonomous(s_Swerve)
        );

    }
}