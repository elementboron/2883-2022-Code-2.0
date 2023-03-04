/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;
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

public class RotateArmMotor extends SubsystemBase 
{

  public WPI_TalonFX motorRotate = new WPI_TalonFX(Constants.Swerve.rotateArmID);
 
  public void TeleOp(DoubleSupplier armRotation) 
  {
      motorRotate.set(armRotation.getAsDouble());
  }

  public void ShoulderSoftLimits()
  {
    motorRotate.configForwardSoftLimitEnable(true);
    motorRotate.configForwardSoftLimitThreshold(0);
    motorRotate.configReverseSoftLimitEnable(true);
    motorRotate.configReverseSoftLimitThreshold(-110*2048);
  }

  public void SetPosition(double desiredPosition, double speed)
  {
    double targetPosition = desiredPosition * 2048;
    if(motorRotate.getSelectedSensorPosition()>targetPosition)
    {
      motorRotate.set((speed*((4*(Math.abs((motorRotate.getSelectedSensorPosition()-targetPosition))/targetPosition)))-0.1));
    } else if (motorRotate.getSelectedSensorPosition()<targetPosition)
    {
      motorRotate.set((-speed*((4*(Math.abs((motorRotate.getSelectedSensorPosition()-targetPosition))/targetPosition)))+0.1));
    }
  }

  public void ArmToZero()
  {
    if(motorRotate.getSelectedSensorPosition()<0)
    {
      motorRotate.set(0.5);
    } 
  }

  public double ShoulderPosition()
  {
    return motorRotate.getSelectedSensorPosition();
  }

  public void UpdateSmartDashNums()
  {

    SmartDashboard.putNumber("RotateMotor Temperature:", motorRotate.getTemperature());
    SmartDashboard.putNumber("RotateMotor Current:", motorRotate.getSupplyCurrent());
    SmartDashboard.putNumber("RotateMotor Rotations:",motorRotate.getSelectedSensorPosition()/2048);

  }
  

  public void Stop()
  {
    motorRotate.set(0);
  }

  public boolean isFinished() 
  {
    return true;
  }
}
