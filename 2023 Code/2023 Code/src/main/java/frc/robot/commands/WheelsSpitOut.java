/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CommonMethodExtensions;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class WheelsSpitOut extends CommandBase
{
    private final GripperWheels s_Wheels;
    
    

    public WheelsSpitOut(GripperWheels subsystem)
    {
        s_Wheels = subsystem;
        
        addRequirements(s_Wheels);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute() 
    {  
        s_Wheels.SpitOut();
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
