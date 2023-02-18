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


public class ExtendArm extends CommandBase
{
    private ArmExtensionMotor s_Arm = new ArmExtensionMotor();
    private DoubleSupplier extensionSpeed;
    private DoubleSupplier rotationSpeed;

    public ExtendArm(ArmExtensionMotor subsystem, DoubleSupplier extension, DoubleSupplier rotation)
    {
        s_Arm = subsystem;
        this.extensionSpeed = extension;
        this.rotationSpeed = rotation;
        
        addRequirements(s_Arm);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute() 
    {  
       s_Arm.Spin(extensionSpeed.getAsDouble(), new WPI_TalonFX(18));
       s_Arm.Spin(Math.pow(rotationSpeed.getAsDouble(),3)/2.3, new WPI_TalonFX(19));
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
