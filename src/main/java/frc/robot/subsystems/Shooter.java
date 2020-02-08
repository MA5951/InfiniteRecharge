/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.*;


/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Shooter extends SubsystemBase {
  private static Shooter shooter;

  private double KP_FLY_WHEEL_SPEED = 0.09;
  private double KI_FLY_WHEEL_SPEED = 0.00004;
  private double KD_FLY_WHEEL_SPEED =0.0001;


  public static double shooterAngle = 25; // TODO
  private double Gravity = 9.807; // TODO

  private double deltaY = 0; // TODO
  private double radiusFlyWheel = 0.0508; // TODO

  private double ticksPerRoundflyWheel = 999.5; // TODO;

  private TalonSRX flyWheelA;
  private TalonSRX flyWheelB;



  public double shootCounter = 0;
  

  private edu.wpi.first.wpilibj.controller.PIDController flyWheelSpeed;

  private Encoder flyWheelEncoder;  

  private Shooter() {
    flyWheelA = new TalonSRX(ShooterConstants.FLY_WHEEL_A);
    flyWheelB = new TalonSRX(ShooterConstants.FLY_WHEEL_B);
    flyWheelB.configOpenloopRamp(0.5);
    flyWheelA.follow(flyWheelB);


    flyWheelSpeed = new edu.wpi.first.wpilibj.controller.PIDController(KP_FLY_WHEEL_SPEED, KI_FLY_WHEEL_SPEED,
        KD_FLY_WHEEL_SPEED);
    flyWheelSpeed.setTolerance(1); // TODO
    flyWheelSpeed.setIntegratorRange(0, 0.01);
    flyWheelEncoder = new Encoder(9 , 8 , false , EncodingType.k4X);

    flyWheelEncoder.setDistancePerPulse(1);

    

  }

  /**
   * Display The given values to the shuffleboard
   */
  public void ShooterValue() {
    SmartDashboard.putNumber("kRateFlyWheelSpeed", kRateFlyWheelSpeed());
    SmartDashboard.putNumber("kRateFlyWheelEncoder", flyWheelEncoder.getRate());
    SmartDashboard.putNumber("kSetPointPID", flyWheelSpeed.getSetpoint());
    //SmartDashboard.putNumber("Output", flyWheelSpeedOutPut(550));
  }

  
   /**
   * Set the power to the fly wheel motors
   * @param speed The given power
   */
  public void controlFlyWheelMotor(double speed) {
    flyWheelB.set(ControlMode.PercentOutput, speed);
  }

  

  /**
   * Set the angle velocity for the fly wheel
   * @return The kRate calculation
   */
  public double kRateFlyWheelSpeed() {
    return (flyWheelEncoder.getRate() * (2 * Math.PI)) / ticksPerRoundflyWheel;
  }



  /**
   * Calculate the PID for the fly wheel
   * @param setPoint The destanation the robot need to reach
   * @return The result of the calculation
   */
  public double flyWheelSpeedOutPut(double setPoint) {
    return MathUtil.clamp(flyWheelSpeed.calculate(kRateFlyWheelSpeed(), setPoint), 0, 0.85);
  }

  /**
   * @param deltaDistance The distance the robot need to reach
   * @return The result of the calculation
   */
  public double calculateSpeedToFlyWheel(double deltaDistance) {
    double radShooterAngle = Math.toRadians(shooterAngle);
    return (2 * (Math.sqrt(((Gravity / 2) * deltaDistance) / 2 * Math.pow(Math.cos(radShooterAngle), 2)
        * (deltaY - (deltaDistance * Math.tan(radShooterAngle)))))) / radiusFlyWheel;
  }

  /**
   * Check if the fly wheel is on the setpoint
   * @return True if on target, false if isn't
   */
  public boolean isFlyWheelOnTraget() {
    return flyWheelSpeed.atSetpoint();
  }



  public static Shooter getinstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }

  /**
   * Count the balls the in and out of the shooter
   */

  @Override
  public void periodic() {
    ShooterValue();
  }
}