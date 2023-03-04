/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CommonMethodExtensions;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class FollowTrajectory extends CommandBase
{
    private final Swerve s_Swerve;
    private final PathPlannerTrajectory traj;
    private final boolean isFirstTime;
    
    

    public FollowTrajectory(Swerve subsystem, PathPlannerTrajectory traj, boolean isFirstTime)
    {
        s_Swerve = subsystem;
        this.traj = traj;
        this.isFirstTime = isFirstTime;
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute() 
    {  
        s_Swerve.followTrajectoryCommand(traj, isFirstTime);
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
