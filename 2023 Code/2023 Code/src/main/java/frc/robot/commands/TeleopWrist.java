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


public class TeleopWrist extends CommandBase
{
    private final WristMotor s_Wrist;
    private final DoubleSupplier wristRotation;
  

    public TeleopWrist(WristMotor subsystem, DoubleSupplier wristRotation)
    {
        s_Wrist = subsystem;
        this.wristRotation = wristRotation;
        
        addRequirements(s_Wrist);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute() 
    {  
        s_Wrist.TeleOpWrist(wristRotation);
     }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
