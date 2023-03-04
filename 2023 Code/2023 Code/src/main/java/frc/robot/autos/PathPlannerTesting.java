package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.StopRobotAutonomous;
//import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.Swerve;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PathPlannerTesting extends SequentialCommandGroup {
    public PathPlannerTesting(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond/2,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared/1.5)
                .setKinematics(Constants.Swerve.swerveKinematics);

        PathPlannerTrajectory traj = PathPlanner.loadPath("TestPath", new PathConstraints(3, 1));    

        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro())
            //new SwerveCommand(s_Swerve, traj, true)
        );
    }
}