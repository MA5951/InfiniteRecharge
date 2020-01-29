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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.ShooterConstants;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Shooter extends SubsystemBase {
  private static Shooter shooter;

  private double KP_FLY_WHEEL_SPEED = 0;
  private double KI_FLY_WHEEL_SPEED = 0;
  private double KD_FLY_WHEEL_SPEED = 0;

  private double KP_SQUISH_MOTOR_SPEED = 0;
  private double KI_SQUISH_MOTOR_SPEED = 0;
  private double KD_SQUISH_MOTOR_SPEED = 0;

  public static double shooterAngle = 0; //TODO
  private double Gravity = 9.8; // TODO

  private double deltaY = 0; //TODO
  private double radiusFlyWheel = 0.0508; // TODO

  private double ticksPerRoundsquishMotor = 1; //TODO;
  private double ticksPerRoundflyWheel = 1; //TODO;

  private TalonSRX flyWheelA;
  private TalonSRX flyWheelB;
  private TalonSRX squishMotor;

  private double kRateSquish = 0; // TODO
  
  private DigitalInput IRBall;
  private Solenoid angleChange;

  private edu.wpi.first.wpilibj.controller.PIDController flyWheelSpeed;
  private edu.wpi.first.wpilibj.controller.PIDController squishMotorSpeed;

  private Shooter(){

  flyWheelA = new TalonSRX(ShooterConstants.FLY_WHEEL_A);
  flyWheelB = new TalonSRX(ShooterConstants.FLY_WHEEL_B);
  flyWheelA.follow(flyWheelB);


  squishMotor = new TalonSRX(ShooterConstants.SQUISH_MOTOR);
  IRBall = new DigitalInput(ShooterConstants.IR_BALL);
  angleChange = new Solenoid(ShooterConstants.ANGLE_CHANGE);

  flyWheelSpeed = new edu.wpi.first.wpilibj.controller.PIDController( KP_FLY_WHEEL_SPEED, KI_FLY_WHEEL_SPEED, KD_FLY_WHEEL_SPEED);
  squishMotorSpeed = new edu.wpi.first.wpilibj.controller.PIDController(KP_SQUISH_MOTOR_SPEED, KI_SQUISH_MOTOR_SPEED, KD_SQUISH_MOTOR_SPEED);
  flyWheelSpeed.setTolerance(1); // TODO
  squishMotorSpeed.setTolerance(1); // TODO
}

  public void ShooterValue(){
    SmartDashboard.putBoolean("IRball", isBallInShooter());
    SmartDashboard.putNumber("kRateFlyWheelSpeed", kRateFlyWheelSpeed());
    SmartDashboard.putNumber("kRateSquishMotor", kRateSquishMotorSpeed());
    SmartDashboard.putBoolean("isPistonOpen", isPistonOpen());
  }

  public boolean isPistonOpen(){
    return angleChange.get();
  }

  public void controlFlyWheelMotor(double speed){
  flyWheelB.set(ControlMode.PercentOutput, speed);
  }

  public void controlSquishMotor(double speed){
    squishMotor.set(ControlMode.PercentOutput, speed);
    }
  
  public void OpenAngleChangeSolenoid(Boolean value){
  angleChange.set(value);
  }
  public double kRateFlyWheelSpeed(){
    return (flyWheelB.getSelectedSensorVelocity() * (2 *Math.PI)) / ticksPerRoundflyWheel;
  }

  public double kRateSquishMotorSpeed(){
    return (squishMotor.getSelectedSensorVelocity() * (2 *Math.PI)) / ticksPerRoundsquishMotor;
  }
  public double flyWheelSpeedOutPut(double setPoint){
  return MathUtil.clamp(flyWheelSpeed.calculate(kRateFlyWheelSpeed(), setPoint), -1, 1);
  }

  public double squishMotorSpeedOutput(){
  return MathUtil.clamp(squishMotorSpeed.calculate(kRateSquishMotorSpeed(), kRateSquish), -1, 1) ;
  }

  public boolean isBallInShooter(){
    return IRBall.get();
  }

  public double calculateSpeedToFlyWheel(double deltaDistance) {
    double radShooterAngle = Math.toRadians(shooterAngle);
  return (2 * (Math.sqrt(((Gravity / 2) * deltaDistance)
   / 2* Math.pow(Math.cos(radShooterAngle), 2)
    * (deltaY - (deltaDistance * Math.tan(radShooterAngle)))))) / radiusFlyWheel;
  }

  public boolean isFlyWheelOnTraget() {
    return flyWheelSpeed.atSetpoint();
  }

  public boolean isSquishOnTarget() {
    return squishMotorSpeed.atSetpoint();
  }

  public static Shooter getinstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }

  @Override
  public void periodic() {
    ShooterValue();
  }
}
