/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class SlowDown extends CommandBase
{
    private final Swerve s_Swerve;

    public SlowDown(Swerve subsystem)
    {
        s_Swerve = subsystem;
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute() 
    {  
        RobotContainer.speedController = RobotContainer.desiredSpeed/2;
     }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
