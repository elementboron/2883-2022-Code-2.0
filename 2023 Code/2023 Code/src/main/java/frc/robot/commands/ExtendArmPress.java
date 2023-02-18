/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class ExtendArmPress extends CommandBase
{
    private ArmExtensionMotor s_Arm = new ArmExtensionMotor();
    private double extensionDistance;
    private double extensionSpeed;

    public ExtendArmPress(ArmExtensionMotor subsystem, double extensionDistance, double extensionSpeed)
    {
        s_Arm = subsystem;
        this.extensionSpeed = extensionDistance;
        this.extensionSpeed = extensionSpeed;
        
        addRequirements(s_Arm);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute() 
    {  
       s_Arm.SpinOut(new WPI_TalonFX(18), extensionSpeed, extensionDistance, true);
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
