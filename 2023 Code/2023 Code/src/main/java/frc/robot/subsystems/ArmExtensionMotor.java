/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommonMethodExtensions;
//import frc.robot.Commands.SpinNeo550;

public class ArmExtensionMotor extends SubsystemBase 
{
  //public final CANSparkMax motor1 = new CANSparkMax(11, MotorType.kBrushless);
  //public final WPI_TalonFX m_indexend = new WPI_TalonFX(8);
  WPI_TalonFX motorExtend = new WPI_TalonFX(18);
  WPI_TalonFX motorRotate = new WPI_TalonFX(19);
  WPI_TalonFX motor;

  /*public void Initialize(WPI_TalonFX motor)  {
    this.motor = motor;
  }**/
 
  public void Spin(double speed, WPI_TalonFX motor1) 
  {
    motor1.set(speed);
  }
  public void SpinIn(WPI_TalonFX motor1, double speed, double distance, boolean tween)
  {
    SpinForInternalIn(motor1, speed, distance, tween);
  }
  
  public void SpinOut(WPI_TalonFX motor1, double speed, double distance, boolean tween)
  {
    SpinForInternalOut(motor1, speed, distance, tween);
  }

  private void SpinForInternalOut(WPI_TalonFX motor1, double speed, double rotations, boolean tween)
  {
    double distance = rotations*2048;
    if(motor1.getSelectedSensorPosition()*2048 < distance )
    {
        if(motor1.getSelectedSensorPosition()*2048 <= distance/2 || !tween)
        {
            motor1.set(speed);
        } else
        {
            double baseSpeed = 0.05;
            double speedMinusBS = speed-baseSpeed;
            double tweenSpeed = CommonMethodExtensions.tweenerReverse(speedMinusBS, (distance/2), (motor1.getSelectedSensorPosition()*2048 - distance/2));
            motor1.set(tweenSpeed+baseSpeed);
        }
    //Out = false;
    } else 
    {
        motor1.set(0);
        //Out = true;
    }
  }

  private void SpinForInternalIn(WPI_TalonFX motor1, double speed, double rotations, boolean tween)
  {
    double distance = rotations*2048;
    if(motor1.getSelectedSensorPosition()*2048 > 0 )
    {
        if(motor1.getSelectedSensorPosition()*2048 >= distance/2 || !tween)
        {
            motor1.set(-speed);
        } else
        {
            double baseSpeed = 0.02;
            double speedMinusBS = speed-baseSpeed;
            double tweenSpeed = CommonMethodExtensions.tweenerReverse(speedMinusBS, (distance/2), (distance/2 - motor1.getSelectedSensorPosition()*2048));
            double actualSpeed = tweenSpeed+baseSpeed;
            motor.set(-actualSpeed);
        }
      //Out = true;
    } else 
    {
        motor1.set(0);
        //Out = false;
    }
  }

  public void UpdateSmartDashNums()
  {
    //SmartDashboard.putNumber("Encoder Position[" + motor.getDeviceID() + "]", encoder.getPosition());
    //SmartDashboard.putNumber("Encoder Velocity[" + motor.getDeviceID() + "]", encoder.getVelocity());
    SmartDashboard.putNumber("ExtendMotor Temperature:", motorExtend.getTemperature());
    SmartDashboard.putNumber("ExtendMotor Current:", motorExtend.getSupplyCurrent());
    SmartDashboard.putNumber("RotateMotor Temperature:", motorRotate.getTemperature());
    SmartDashboard.putNumber("RotateMotor Current:", motorRotate.getSupplyCurrent());
    SmartDashboard.putNumber("RotateMotor Rotations:",motorRotate.getSelectedSensorPosition()/2048);
    SmartDashboard.putNumber("ExtendMotor Rotations:", motorExtend.getSelectedSensorPosition()/2048);
    //SmartDashboard.putNumber("Max Distance[" + motor.getDeviceID() + "]", maxDistance);
  }
  

  public void Stop()
  {
    motor.set(0);
  }

  public boolean isFinished() 
  {
    return true;
  }
}
