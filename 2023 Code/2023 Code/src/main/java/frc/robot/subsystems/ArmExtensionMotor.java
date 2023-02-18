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
  WPI_TalonFX motor;

  public void Initialize(WPI_TalonFX motor)  {
    this.motor = motor;
  }
 
  public void Spin(double speed) 
  {
    motor.set(speed);
  }

  /*public void SpinIn(double speed, double distance, boolean tween)
  {
    SpinForInternalIn(speed, distance, tween);
  }
  
  public void SpinOut(double speed, double distance, boolean tween)
  {
    SpinForInternalOut(speed, distance, tween);
  }

  private void SpinForInternalOut(double speed, double distance, boolean tween)
  {
    if(encoder.getPosition() < distance )
    {
        if(encoder.getPosition() <= distance/2 || !tween)
        {
            motor.set(speed);
        } else
        {
            double baseSpeed = 0.05;
            double speedMinusBS = speed-baseSpeed;
            double tweenSpeed = CommonMethodExtensions.tweenerReverse(speedMinusBS, (distance/2), (encoder.getPosition() - distance/2));
            motor.set(tweenSpeed+baseSpeed);
        }
    //Out = false;
    } else 
    {
        motor.set(0);
        //Out = true;
    }
  }

  private void SpinForInternalIn(double speed, double distance, boolean tween)
  {
    if(encoder.getPosition() > 0 )
    {
        if(encoder.getPosition() >= distance/2 || !tween)
        {
            motor.set(-speed);
        } else
        {
            double baseSpeed = 0.02;
            double speedMinusBS = speed-baseSpeed;
            double tweenSpeed = CommonMethodExtensions.tweenerReverse(speedMinusBS, (distance/2), (distance/2 - encoder.getPosition()));
            double actualSpeed = tweenSpeed+baseSpeed;
            motor.set(-actualSpeed);
        }
      //Out = true;
    } else 
    {
        motor.set(0);
        //Out = false;
    }
  }
**/
  public void UpdateSmartDashNums()
  {
    //SmartDashboard.putNumber("Encoder Position[" + motor.getDeviceID() + "]", encoder.getPosition());
    //SmartDashboard.putNumber("Encoder Velocity[" + motor.getDeviceID() + "]", encoder.getVelocity());
    SmartDashboard.putNumber("Motor Temperature[" + motor.getDeviceID() + "]", motor.getTemperature());
    SmartDashboard.putNumber("Motor Current[" + motor.getDeviceID() + "]", motor.getSupplyCurrent());
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
