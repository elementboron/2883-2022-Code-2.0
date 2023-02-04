package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.LimelightReader;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightReader.BotPoseInfoHolder;

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

public class DriveToAprilTag extends SequentialCommandGroup {
    public DriveToAprilTag(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond/2,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        double[] currentBotPose = LimelightReader.Instance().botPose.getDoubleArray(new double[3]);
        while(currentBotPose.length < 5)
        {
            currentBotPose = LimelightReader.Instance().botPose.getDoubleArray(currentBotPose);
        }
        double[] distancePose = LimelightReader.GetDistanceVector(currentBotPose[0], currentBotPose[1], currentBotPose[5], LimelightReader.April3Pose[0], LimelightReader.April3Pose[1], 0);
        //BotPoseInfoHolder EasyToUseBoy = new BotPoseInfoHolder(distancePose);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0) ),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(distancePose[0],distancePose[1], new Rotation2d(distancePose[2])),
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

        WaitCommand wait = new WaitCommand(5);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            swerveControllerCommand
        );
    }
}