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
  
  WPI_TalonFX motor;

  /*public void Initialize(WPI_TalonFX motor)  {
    this.motor = motor;
  }**/
 
  public void TeleOpWrist(DoubleSupplier wristRotation) 
  {
    wristMotor.set(wristRotation.getAsDouble());

  }

  public void setWristRotation(double speed, double desiredPosition)
  {
    if(encoder.getPosition()>desiredPosition){

      InternalSpinWristReverse(speed, desiredPosition);

    } else if (encoder.getPosition()<desiredPosition){

      InternalSpinWristForward(speed, desiredPosition);

    } else if (encoder.getPosition() == desiredPosition){

      Stop();

    }

  }

  public void InternalSpinWristForward(double speed, double distance)
  {
    if(encoder.getPosition()<distance)
    {
      wristMotor.set(speed);
    } else {
      wristMotor.set(0);
    }
  }

  public void InternalSpinWristReverse(double speed, double distance)
  {
    if(encoder.getPosition()<distance)
    {
      wristMotor.set(-speed);
    } else {
      wristMotor.set(0);
    }
  }

  public void UpdateSmartDashNums()
  {

    SmartDashboard.putNumber("WristMotor Temperature:", wristMotor.getMotorTemperature());
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
