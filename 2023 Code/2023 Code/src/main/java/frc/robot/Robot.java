// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.RotateArmMotor;
//import frc.robot.subsystems.GripperWheels;
//import frc.robot.subsystems.Pneumatics;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final RotateArmMotor s_Arm = new RotateArmMotor();
  //private final Pneumatics s_Pneumatics = new Pneumatics();
  //public final Solenoid gripper = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Swerve.gripperSolenoidID);

  //CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
  //CANSparkMax motor2 = new CANSparkMax(7, MotorType.kBrushless);


  public boolean toggle = false;
  boolean lastFramespinOutValue = false;

  public static CANSparkMax wristMotor = new CANSparkMax(Constants.Swerve.wristRotationID, MotorType.kBrushless);
  public static CANSparkMax wheelsMotor = new CANSparkMax(Constants.Swerve.wheelsMotorID, MotorType.kBrushless);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    
    ctreConfigs = new CTREConfigs();
    //s_Arm.Initialize(new WPI_TalonFX(18));
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    LimelightReader.Instance().UpdateLimeCam();
    LimelightReader.Instance().UpdateSmartDashboard();
    /*NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 90.0;
    
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 31.5;
    
    // distance from the target to the floor
    double goalHeightInches = 25.5;
    
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
      
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Estimated Distance in Inches", distanceFromLimelightToGoalInches);
    s_Arm.UpdateSmartDashNums();

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.*/
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //motor2.set((m_stick.getRawAxis(wristRotationAxisPositive)-m_stick.getRawAxis(wristRotationAxisNegative))/1.5);
    //motor.set(m_stick.getRawAxis(wheelRotationAxis));

  
/*
    boolean pressedThisFrame = spinOutButton.getAsBoolean();

    if(lastFramespinOutValue != pressedThisFrame && pressedThisFrame)
    {
      toggle = !toggle;
    }
    lastFramespinOutValue = pressedThisFrame;

   if(toggle)
    {
      s_Arm.SpinIn(new WPI_TalonFX(18), 0.2, 10, true);
    }
    else
    {
      s_Arm.SpinOut(new WPI_TalonFX(18), 0.2, 10, true);
    }
  **/

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
