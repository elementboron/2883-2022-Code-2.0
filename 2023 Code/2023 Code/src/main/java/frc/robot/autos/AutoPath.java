package frc.robot.autos;

import frc.robot.Constants;
//import frc.robot.autos.Actions.Action;
//import frc.robot.autos.Actions.WaitAction;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

import java.util.List;


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

public class AutoPath extends SequentialCommandGroup {
    public AutoPath(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        final Translation2d translation = new Translation2d(0,0);

        // An example trajectory to follow.  All units in meters.
        String file_path = "paths/ObstacleCourse.path";
        Trajectory exampleTrajectory = AutoTrajectoryReader.generateTrajectoryFromFile(file_path, config);
        String file_path_test = "paths/Test.path";
        Trajectory exampleTrajectory1 = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_test, config);
        Trajectory stopTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                config);


        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommand2 =
            new SwerveControllerCommand(
                exampleTrajectory1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
                    
        SwerveControllerCommand stopCommand =
            new SwerveControllerCommand(
                stopTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        WaitCommand wait = new WaitCommand(5);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            swerveControllerCommand,
            stopCommand,
            wait,
            swerveControllerCommand2      
        );

    }
}