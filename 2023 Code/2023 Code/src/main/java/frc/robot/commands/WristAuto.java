/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class WristAuto extends CommandBase
{
    private final RotateArmMotor s_Arm;
  

    public WristAuto(RotateArmMotor subsystem)
    {
        s_Arm = subsystem;
        
        addRequirements(s_Arm);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute() 
    {  
        s_Arm.setWristRotation(0.1, 5);
     }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
