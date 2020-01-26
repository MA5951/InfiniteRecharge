/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Shooter extends SubsystemBase {
private static Shooter shooter;

private double KP_FLY_WHEEL_SPEED =0;
private double KI_FLY_WHEEL_SPEED = 0;
private double KD_FLY_WHEEL_SPEED = 0;

private double KP_SQUIS_MOTOR_SPEED = 0;
private double KI_SQUIS_MOTOR_SPEED = 0;
private double KD_SQUIS_MOTOR_SPEED = 0;

private double KrateSquis = 0;

private TalonSRX flyWheelA;
private TalonSRX flyWheelB;
private TalonSRX squishMotor;
private DigitalInput IRBall;
private Solenoid angleChange;
private edu.wpi.first.wpilibj.controller.PIDController flyWheelSpeed;
private edu.wpi.first.wpilibj.controller.PIDController squishMotorSpeed;

private Shooter(){
flyWheelA = new TalonSRX(Constants.FLY_WHEELA);
flyWheelB = new TalonSRX(Constants.FLY_WHEELB);
flyWheelA.follow(flyWheelB);


squishMotor = new TalonSRX(Constants.SQUISH_MOTOR);
IRBall = new DigitalInput(Constants.IR_BALL);
angleChange = new Solenoid(Constants.ANGLE_CHANGE);

flyWheelSpeed = new edu.wpi.first.wpilibj.controller.PIDController( KP_FLY_WHEEL_SPEED, KI_FLY_WHEEL_SPEED, KD_FLY_WHEEL_SPEED);
squishMotorSpeed = new edu.wpi.first.wpilibj.controller.PIDController(KP_SQUIS_MOTOR_SPEED, KI_SQUIS_MOTOR_SPEED, KD_SQUIS_MOTOR_SPEED);
}

public void value(){
  SmartDashboard.putBoolean("IRBALL", isABallInTheShooter());
  SmartDashboard.putNumber("kRateFlyWheelSpeed", kRateFlyWheelSpeed());
  SmartDashboard.putNumber("kRateFlyWheelSpeed", kRateFlyWheelSpeed());
}
public void controlFlyWheelMotor(double speed){
flyWheelB.set(ControlMode.PercentOutput, speed);
}
public void OpenAngleChangeSolenoid(Boolean value){
angleChange.set(value);
}
public double kRateFlyWheelSpeed(){
  return flyWheelB.getSelectedSensorVelocity();
}

public double kRateSquishMotorSpeed(){
  return squishMotor.getSelectedSensorVelocity();
}
public double flyWheelSpeedOutPut(double setPoint){
return flyWheelSpeed.calculate(kRateFlyWheelSpeed(), setPoint);
}

public double squishMotorSpeedOutPut(double setPoint){
 return squishMotorSpeed.calculate(kRateSquishMotorSpeed(), KrateSquis);
}

public boolean isABallInTheShooter(){
  return IRBall.get();
}

public int calculateSpeedToFlyWheel(double distance){
return 1; //TODO
}

  public static Shooter getinstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }

  @Override
  public void periodic() {
    value();
  }


}
