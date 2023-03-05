package frc.robot.autos;

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
import frc.robot.Constants;
import frc.robot.commands.ArmToHigh;
import frc.robot.commands.ArmToHome;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.RotateAuto;
import frc.robot.commands.StopRobotAutonomous;
import frc.robot.commands.StopWrist;
import frc.robot.commands.WheelsSpitOut;
import frc.robot.commands.WheelsStop;
import frc.robot.commands.WheelsSuckIn;
import frc.robot.commands.WristToDown;
import frc.robot.commands.WristToHigh;
import frc.robot.commands.WristToHome;
import frc.robot.subsystems.GripperWheels;
import frc.robot.subsystems.RotateArmMotor;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristMotor;

public class RotateTesting extends SequentialCommandGroup {

    
    public RotateTesting(Swerve s_Swerve, RotateArmMotor s_Arm, WristMotor s_Wrist, GripperWheels s_Wheels){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    (Constants.AutoConstants.kMaxSpeedMetersPerSecond),
                    (Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared))
                .setKinematics(Constants.Swerve.swerveKinematics);

        var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

        //PathPlannerTrajectory traj = PathPlanner.loadPath("DriveStraighPath", new PathConstraints(3, 1));
        Trajectory traj =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4, 0, new Rotation2d(0)),
                config);
        Trajectory traj2 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0.25, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0.5, 0, new Rotation2d(0)),
                config);
        Trajectory traj3 =
                TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(0, 0.5)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(0, 2, new Rotation2d(0)),
                    config);
        Trajectory traj4 =
                    TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(0.5, 0)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(1, 0, new Rotation2d(0)),
                        config);

        SwerveControllerCommand DriveCommand =
            new SwerveControllerCommand(
                traj,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
        SwerveControllerCommand PickUpCommand =
            new SwerveControllerCommand(
                traj2,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
                
        SwerveControllerCommand ToDockCommand =
                new SwerveControllerCommand(
                    traj3,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

        SwerveControllerCommand OnDockCommand =
                    new SwerveControllerCommand(
                        traj4,
                        s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                        thetaController,
                        s_Swerve::setModuleStates,
                        s_Swerve);

        addCommands(
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new ArmToHigh(s_Arm),
            //new WristToHigh(s_Wrist),
            //new StopWrist(s_Wrist),
            new WaitCommand(1),
            //new WheelsSpitOut(s_Wheels),
            new WaitCommand(1),
            //new WheelsStop(s_Wheels),
            //new WristToHome(s_Wrist),
            new ArmToHome(s_Arm),
            new StopRobotAutonomous(s_Swerve),
            new RotateAuto(s_Swerve, 180),
            new StopRobotAutonomous(s_Swerve),
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new InstantCommand(() -> s_Swerve.resetOdometry(traj.getInitialPose())),
            new WaitCommand(1),

            DriveCommand,
            new RotateAuto(s_Swerve, 0),
            new StopRobotAutonomous(s_Swerve),
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new InstantCommand(() -> s_Swerve.resetOdometry(traj.getInitialPose())),
            new WaitCommand(2),

            //new WristToDown(s_Wrist),
            //new StopWrist(s_Wrist),
            //new WheelsSuckIn(s_Wheels),
            PickUpCommand,
            new WaitCommand(2),
            new RotateAuto(s_Swerve, 0),
            new StopRobotAutonomous(s_Swerve),
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new InstantCommand(() -> s_Swerve.resetOdometry(traj.getInitialPose())),
            new WaitCommand(2),
    
            ToDockCommand,
            new WaitCommand(1),
            new RotateAuto(s_Swerve, 180),
            new StopRobotAutonomous(s_Swerve),
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new InstantCommand(() -> s_Swerve.resetOdometry(traj4.getInitialPose())),
            new WaitCommand(1),
            OnDockCommand
            //new AutoBalance(s_Swerve)
            //new WheelsStop(s_Wheels),
            //new WristToHome(s_Wrist)
            //new RotateAuto(s_Swerve, 180),
            //new InstantCommand((() -> s_Swerve.zeroGyro()))

        );

    }
}