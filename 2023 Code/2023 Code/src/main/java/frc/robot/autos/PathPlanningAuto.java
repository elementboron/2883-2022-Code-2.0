package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.FollowTrajectory;
//import frc.robot.autos.Actions.Action;
//import frc.robot.autos.Actions.WaitAction;
import frc.robot.commands.TeleopSwerve;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.autos.Actions.WaitAction;
import frc.robot.subsystems.*;

public class PathPlanningAuto extends SequentialCommandGroup {
    public PathPlanningAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
    
        PathPlannerTrajectory traj = PathPlanner.loadPath("TestPath", new PathConstraints(3, 1)); 


        addCommands(
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new FollowTrajectory(s_Swerve, traj, true)
        );

    }
}