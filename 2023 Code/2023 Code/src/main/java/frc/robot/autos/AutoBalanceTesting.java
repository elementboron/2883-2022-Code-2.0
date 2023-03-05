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

public class AutoBalanceTesting extends SequentialCommandGroup {

    
    public AutoBalanceTesting(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    (Constants.AutoConstants.kMaxSpeedMetersPerSecond/1.5),
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
                List.of(new Translation2d(0.5, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0.75, 0, new Rotation2d(0)),
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


        addCommands(
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new InstantCommand(() -> s_Swerve.resetOdometry(traj.getInitialPose())),
            DriveCommand,
            new WaitCommand(2),
            new AutoBalance(s_Swerve)
        );

    }
}