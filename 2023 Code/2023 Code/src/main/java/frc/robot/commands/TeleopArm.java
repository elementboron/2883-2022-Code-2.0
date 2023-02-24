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


public class TeleopArm extends CommandBase
{
    private final RotateArmMotor s_Arm;
    private final DoubleSupplier armRotation;
    private final DoubleSupplier wristRotation;
    private final boolean jointMovement;    

    public TeleopArm(RotateArmMotor subsystem, DoubleSupplier armRotation, DoubleSupplier wristRotation, boolean jointMovement)
    {
        s_Arm = subsystem;
        this.armRotation = armRotation;
        this.wristRotation = wristRotation;
        this.jointMovement = jointMovement;
        
        addRequirements(s_Arm);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute() 
    {  
        s_Arm.TeleOp(armRotation, wristRotation, jointMovement);
     }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
