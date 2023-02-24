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


public class TeleopWheels extends CommandBase
{
    private final GripperWheels s_Wheels;
    private final DoubleSupplier wheelSpeedPositive;
    private final DoubleSupplier wheelSpeedNegative;
  

    public TeleopWheels(GripperWheels subsystem, DoubleSupplier wheelSpeedPositive, DoubleSupplier wheelSpeedNegative)
    {
        s_Wheels = subsystem;
        this.wheelSpeedPositive = wheelSpeedPositive;
        this.wheelSpeedNegative = wheelSpeedNegative;
        
        addRequirements(s_Wheels);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute() 
    {  
        s_Wheels.Drive(wheelSpeedPositive, wheelSpeedNegative);
     }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
