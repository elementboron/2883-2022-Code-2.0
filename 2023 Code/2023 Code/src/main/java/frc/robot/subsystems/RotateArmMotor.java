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

public class RotateArmMotor extends SubsystemBase 
{

  WPI_TalonFX motorRotate = new WPI_TalonFX(Constants.Swerve.rotateArmID);
 
  public void TeleOp(DoubleSupplier armRotation) 
  {
    motorRotate.set(armRotation.getAsDouble());
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
