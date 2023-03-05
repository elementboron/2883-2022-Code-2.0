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

public class GripperWheels extends SubsystemBase 
{
  //public final CANSparkMax motor1 = new CANSparkMax(11, MotorType.kBrushless);
  //public final WPI_TalonFX m_indexend = new WPI_TalonFX(8);
  //WPI_TalonFX motorExtend = new WPI_TalonFX(Constants.Swerve.extendMotorID);
  CANSparkMax wheelMotor = Robot.wheelsMotor;
  RelativeEncoder encoder = wheelMotor.getEncoder();

  /*public void Initialize(WPI_TalonFX motor)  {
    this.motor = motor;
  }**/


  public void Drive(DoubleSupplier positiveRotation, DoubleSupplier negativeRotation)
  {
    wheelMotor.set((positiveRotation.getAsDouble()-negativeRotation.getAsDouble()));
  }

  public void Stop()
  {
    wheelMotor.set(0);
  }

  public void SuckIn()
  {
    wheelMotor.set(1);
  }

  public void SpitOut()
  {
    wheelMotor.set(-1);
  }

  public boolean isFinished() 
  {
    return true;
  }
}
