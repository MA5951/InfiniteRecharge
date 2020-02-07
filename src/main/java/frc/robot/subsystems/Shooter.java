/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
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

  private double KP_FLY_WHEEL_SPEED = 0;
  private double KI_FLY_WHEEL_SPEED = 0;
  private double KD_FLY_WHEEL_SPEED = 0;

  private double KP_SQUISH_MOTOR_SPEED = 0;
  private double KI_SQUISH_MOTOR_SPEED = 0;
  private double KD_SQUISH_MOTOR_SPEED = 0;

  public static double shooterAngle = 25; // TODO
  private double Gravity = 9.8; // TODO

  private double deltaY = 0; // TODO
  private double radiusFlyWheel = 0.0508; // TODO

  private double ticksPerRoundsquishMotor = 1; // TODO;
  private double ticksPerRoundflyWheel = 1; // TODO;

  private TalonSRX flyWheelA;
  private TalonSRX flyWheelB;
  private TalonSRX squishMotor;

  private double kRateSquish = 0; // TODO

  private DigitalInput IRBall;
  private Solenoid angleChange;

  public double shootCounter = 0;
  private boolean lastState;

  private edu.wpi.first.wpilibj.controller.PIDController flyWheelSpeed;
  private edu.wpi.first.wpilibj.controller.PIDController squishMotorSpeed;

  private Encoder flyWheelEncoder;
  private Encoder squishMotorEncoder;


  private Shooter() {

    flyWheelA = new TalonSRX(ShooterConstants.FLY_WHEEL_A);
    flyWheelB = new TalonSRX(ShooterConstants.FLY_WHEEL_B);
    flyWheelA.follow(flyWheelB);

    squishMotor = new TalonSRX(ShooterConstants.SQUISH_MOTOR);
    IRBall = new DigitalInput(ShooterConstants.IR_BALL);
    angleChange = new Solenoid(ShooterConstants.ANGLE_CHANGE);

    flyWheelSpeed = new edu.wpi.first.wpilibj.controller.PIDController(KP_FLY_WHEEL_SPEED, KI_FLY_WHEEL_SPEED,
        KD_FLY_WHEEL_SPEED);
    squishMotorSpeed = new edu.wpi.first.wpilibj.controller.PIDController(KP_SQUISH_MOTOR_SPEED, KI_SQUISH_MOTOR_SPEED,
        KD_SQUISH_MOTOR_SPEED);
    flyWheelSpeed.setTolerance(1); // TODO
    squishMotorSpeed.setTolerance(1); // TODO

    flyWheelEncoder = new Encoder(9 , 8 , false , EncodingType.k4X);
    squishMotorEncoder = new Encoder (6 ,7 , false , EncodingType.k4X);

    flyWheelEncoder.setDistancePerPulse(1);
    squishMotorEncoder.setDistancePerPulse(1);

    lastState = IRBall.get();
  }

  /**
   * Display The given values to the shuffleboard
   */
  public void ShooterValue() {
    SmartDashboard.putBoolean("IRball", !IRBall.get());
    SmartDashboard.putNumber("kRateFlyWheelSpeed", kRateFlyWheelSpeed());
    SmartDashboard.putNumber("kRateSquishMotor", kRateSquishMotorSpeed());
    SmartDashboard.putBoolean("isPistonOpen", isPistonOpen());
    SmartDashboard.putNumber("ball", shootCounter);
    
  }

  /**
   * Check if shooter piston is open
   * @return True if open and false if isn't
   */
  public boolean isPistonOpen() {
    return angleChange.get();
  }

   /**
   * Set the power to the fly wheel motors
   * @param speed The given power
   */
  public void controlFlyWheelMotor(double speed) {
    flyWheelB.set(ControlMode.PercentOutput, speed);
  }

   /**
   * Set the power to the squish motors
   * @param speed The given power
   */
  public void controlSquishMotor(double speed) {
    squishMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set the intake piston to open
   * @param value True if want to open and false if doesn't
   */
  public void OpenAngleChangeSolenoid(Boolean value) {
    angleChange.set(value);
  }

  /**
   * Set the angle velocity for the fly wheel
   * @return The kRate calculation
   */
  public double kRateFlyWheelSpeed() {
    return (flyWheelEncoder.getRate() * (2 * Math.PI)) / ticksPerRoundflyWheel;
  }

  /**
   * Set the angle speed for the squish
   * @return The kRate calculation
   */
  public double kRateSquishMotorSpeed() {
    return (squishMotorEncoder.getRate() * (2 * Math.PI)) / ticksPerRoundsquishMotor;
  }

  /**
   * Calculate the PID for the fly wheel
   * @param setPoint The destanation the robot need to reach
   * @return The result of the calculation
   */
  public double flyWheelSpeedOutPut(double setPoint) {
    return MathUtil.clamp(flyWheelSpeed.calculate(kRateFlyWheelSpeed(), setPoint), -1, 1);
  }

  /**
   * Calculate the PID for the squish
   * @return The result of the calculation
   */
  public double squishMotorSpeedOutput() {
    return MathUtil.clamp(squishMotorSpeed.calculate(kRateSquishMotorSpeed(), kRateSquish), -1, 1);
  }

  /**
   * Check if the ball is in the shooter 
   * @return True if IR is "seeing" something and flase if isn't
   */
  public boolean isBallInShooter() {
    return IRBall.get();
  }

  /**
   * Calculate the needed speed of the fly wheel to shoot the ball
   * @param deltaDistance The disired distance
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

  /**
   * Check if the squish is on the setpoint
   * @return True if on target, false if isn't
   */
  public boolean isSquishOnTarget() {
    return squishMotorSpeed.atSetpoint();
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
  public void ballCounter() {
    if (IRBall.get() != lastState) {
      shootCounter += 0.5;
      lastState = IRBall.get();
    }
  }

  @Override
  public void periodic() {
    ShooterValue();
    ballCounter();
  }
}