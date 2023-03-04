/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import javax.sql.rowset.WebRowSet;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommonMethodExtensions;
import frc.robot.Constants;
//import frc.robot.Commands.SpinNeo550;
import frc.robot.Robot;

public class WristMotor extends SubsystemBase 
{
  //public final CANSparkMax motor1 = new CANSparkMax(11, MotorType.kBrushless);
  //public final WPI_TalonFX m_indexend = new WPI_TalonFX(8);
  //WPI_TalonFX motorExtend = new WPI_TalonFX(Constants.Swerve.extendMotorID);
  CANSparkMax wristMotor = Robot.wristMotor;
  RelativeEncoder encoder = wristMotor.getEncoder();
  
 
  public void TeleOpWrist(DoubleSupplier wristRotation) 
  {
    wristMotor.set(wristRotation.getAsDouble());
  }

  public void ToPosition(double desiredPosition, double speed)
  {
    if(encoder.getPosition()<desiredPosition)
    {
      wristMotor.set(speed);
    } else if(encoder.getPosition()>desiredPosition)
    {
      wristMotor.set(-speed);
    }
  }

  public void ToHome()
  {
    if(encoder.getPosition()<0)
    {
      wristMotor.set(0.4);
    }
  }

  public double WristPosition(){
    return encoder.getPosition();
  }

  public void SetWristSoftLimits()
  {
    encoder.setPosition(0);
    wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
  }


  public void UpdateSmartDashNums()
  {

    SmartDashboard.putNumber("WristMotor Rotations:", encoder.getPosition());
  }
  

  public void Stop()
  {
    wristMotor.set(0);
  }

  public boolean isFinished() 
  {
    return true;
  }
}
