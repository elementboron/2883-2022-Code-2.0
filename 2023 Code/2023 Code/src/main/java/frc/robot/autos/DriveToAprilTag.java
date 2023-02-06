package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.autos.Actions.RotateUntilPose;
import frc.robot.subsystems.LimelightReader;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightReader.BotPoseInfoHolder;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
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
    private Swerve s_Swerve;
    SwerveControllerCommand InteriorCommand;
    SwerveControllerCommand AprilCommand;
    Trajectory trajectoryInterior;
    Trajectory trajectoryApril;

    public DriveToAprilTag(Swerve s_Swerve){
        this.s_Swerve=s_Swerve;
        RotateUntilPose rotboy = new RotateUntilPose();

        GenerateCurrentTrajectory();
        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectoryInterior.getInitialPose())),
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            InteriorCommand
           // rotboy,
           /*  new InstantCommand(() -> GenerateCurrentTrajectory()),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectoryApril.getInitialPose())),
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            AprilCommand**/


        );
    }



    public void GenerateCurrentTrajectory(){

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

        double[] distancePose = LimelightReader.GetDistanceVector(currentBotPose[0], currentBotPose[1], 0, LimelightReader.April3Pose[0], LimelightReader.April3Pose[1], 0);
        double[] waypoinRightInteriorDistance = LimelightReader.GetDistanceVector(currentBotPose[0], currentBotPose[1], 0, LimelightReader.DockWayPointRightRedInterior[0], LimelightReader.DockWayPointRightRedInterior[1], 0);
        double[] waypoinRightExteriorDistance = LimelightReader.GetDistanceVector(currentBotPose[0], currentBotPose[1], 0, LimelightReader.DockWayPointRightRedExterior[0], LimelightReader.DockWayPointRightRedExterior[1], 0);       
       

        trajectoryInterior =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(currentBotPose[2]) ),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-waypoinRightExteriorDistance[0], -waypoinRightExteriorDistance[1])),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-distancePose[0],-distancePose[1], new Rotation2d(0)),
            config);

            trajectoryApril =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(currentBotPose[2]) ),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-distancePose[0]/2, -distancePose[1]/2)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-distancePose[0],-distancePose[1], new Rotation2d(0)),
                config);
        
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        InteriorCommand =
        new SwerveControllerCommand(
            trajectoryInterior,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

        AprilCommand =
        new SwerveControllerCommand(
            trajectoryApril,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    }


}