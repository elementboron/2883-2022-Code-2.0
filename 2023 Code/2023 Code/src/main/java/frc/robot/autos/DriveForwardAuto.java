package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.StopRobotAutonomous;
import frc.robot.subsystems.Swerve;

import java.util.List;

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

public class DriveForwardAuto extends SequentialCommandGroup {
    public DriveForwardAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond/2,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared/1.5)
                .setKinematics(Constants.Swerve.swerveKinematics);

        

        // An example trajectory to follow.  All units in meters.
        /*Trajectory driveForward =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                config);

        Trajectory driveBackward =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0.5)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand driveForwardCommand =
            new SwerveControllerCommand(
                driveForward,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand driveBackwardCommand =
            new SwerveControllerCommand(
                driveBackward,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);**/


        addCommands(
            // InstantCommand(() -> s_Swerve.resetOdometry(driveForward.getInitialPose())),
            new InstantCommand(() -> s_Swerve.zeroGyro())
            //driveForwardCommand
            //driveBackwardCommand
        );
    }
}