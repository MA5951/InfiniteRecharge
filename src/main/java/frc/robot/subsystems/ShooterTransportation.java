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

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.commands.Shooter.PIDFlyWheel;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class ShooterTransportation extends SubsystemBase {
  private static ShooterTransportation shooterTransportation;

  private double KP_SQUISH_MOTOR_SPEED = 0.0093;
  private double KI_SQUISH_MOTOR_SPEED = 0.0025;
  private double KD_SQUISH_MOTOR_SPEED = 0;

  private double ticksPerRoundsquishMotor = 919.25; // TODO;
  private TalonSRX squishMotor;

  private double kRateSquish = -90; // TODO

  private DigitalInput IRBall;

  public double shootCounter = 0;
  private boolean lastState;

  private edu.wpi.first.wpilibj.controller.PIDController squishMotorSpeed;

  private Encoder squishMotorEncoder;

  private ShooterTransportation() {

    squishMotor = new TalonSRX(ShooterConstants.SQUISH_MOTOR);
    IRBall = new DigitalInput(ShooterConstants.IR_BALL);

    squishMotorSpeed = new edu.wpi.first.wpilibj.controller.PIDController(KP_SQUISH_MOTOR_SPEED, KI_SQUISH_MOTOR_SPEED,
        KD_SQUISH_MOTOR_SPEED);
    squishMotorSpeed.setTolerance(1); // TODO

    squishMotorSpeed.setIntegratorRange(-0.85, 0);

    squishMotorEncoder = new Encoder(6, 7, true, EncodingType.k4X);

    squishMotorEncoder.setDistancePerPulse(1);

    lastState = IRBall.get();
  }

  /**
   * Display The given values to the shuffleboard
   */
  public void ShooterValue() {
    SmartDashboard.putBoolean("IRball", !IRBall.get());
    SmartDashboard.putNumber("kRateSquishMotor", kRateSquishMotorSpeed());
    SmartDashboard.putNumber("ball", shootCounter);
    SmartDashboard.putNumber("kSetPointPIDSquish", squishMotorSpeed.getSetpoint());
    SmartDashboard.putNumber("squishMotorCurrent", squishMotor.getStatorCurrent());
    SmartDashboard.putNumber("BallCounter", shootCounter);
  }

  public double getMotorCurrnet() {
    return squishMotor.getStatorCurrent();
  }

  /**
   * Set the power to the squish motors
   * 
   * @param speed The given power
   */
  public void controlSquishMotor(double speed) {
    squishMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set the angle speed for the squish
   * 
   * @return The kRate calculation
   */
  public double kRateSquishMotorSpeed() {
    return (squishMotorEncoder.getRate() * (2 * Math.PI)) / ticksPerRoundsquishMotor;
  }

  /**
   * Calculate the PID for the squish
   * 
   * @return The result of the calculation
   */
  public double squishMotorSpeedOutput() {
    return MathUtil.clamp(squishMotorSpeed.calculate(kRateSquishMotorSpeed(), kRateSquish), -0.8, 0);
  }

  /**
   * Check if the ball is in the shooter
   * 
   * @return True if IR is "seeing" something and flase if isn't
   */
  public boolean isBallInShooter() {
    return IRBall.get();
  }

  /**
   * Check if the squish is on the setpoint
   * 
   * @return True if on target, false if isn't
   */
  public boolean isSquishOnTarget() {
    return squishMotorSpeed.atSetpoint();
  }

  public static ShooterTransportation getinstance() {
    if (shooterTransportation == null) {
      shooterTransportation = new ShooterTransportation();
    }
    return shooterTransportation;
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