package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.Swerve;

public class PathPlanningAuto extends SequentialCommandGroup {
    public PathPlanningAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
    
        PathPlannerTrajectory traj = PathPlanner.loadPath("Test Path", new PathConstraints(3, 1));

    



        addCommands(
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new FollowTrajectory(s_Swerve, traj, true)
        );

    }
}