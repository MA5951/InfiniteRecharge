/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.Constants.*;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */

public class Shooter extends SubsystemBase {
  private static Shooter shooter;

  private double KP_FLY_WHEEL_SPEED = 5e-4;
  private double KI_FLY_WHEEL_SPEED = 5e-5;
  private double KD_FLY_WHEEL_SPEED = 0;

  private CANSparkMax flyWheelA;
  private CANSparkMax flyWheelB;
  private CANEncoder flyWheelEncoderA;
  private CANEncoder flyWheelEncoderB;

  public double shootCounter = 0;
  private edu.wpi.first.wpilibj.controller.PIDController flyWheelSpeed;

  private Shooter() {
    flyWheelA = new CANSparkMax(ShooterConstants.FLY_WHEEL_A, MotorType.kBrushless);
    flyWheelB = new CANSparkMax(ShooterConstants.FLY_WHEEL_B, MotorType.kBrushless);

    flyWheelA.setSmartCurrentLimit(60);
    flyWheelA.setIdleMode(IdleMode.kCoast);

    flyWheelB.setSmartCurrentLimit(60);
    flyWheelB.setIdleMode(IdleMode.kCoast);

    flyWheelEncoderA = flyWheelA.getEncoder();
    flyWheelEncoderA.setVelocityConversionFactor(1);
    flyWheelEncoderA.setPositionConversionFactor(1);

    flyWheelEncoderB = flyWheelB.getEncoder();
    flyWheelEncoderB.setVelocityConversionFactor(1);
    flyWheelEncoderB.setPositionConversionFactor(1);
    flyWheelA.setInverted(true);
    flyWheelB.follow(flyWheelA);
    flyWheelA.setOpenLoopRampRate(0);

    flyWheelSpeed = new edu.wpi.first.wpilibj.controller.PIDController(KP_FLY_WHEEL_SPEED, KI_FLY_WHEEL_SPEED,
        KD_FLY_WHEEL_SPEED);

    flyWheelSpeed.setTolerance(70);

  }

  /**
   * Display The given values to the shuffleboard
   */
  public void ShooterValue() {
    SmartDashboard.putNumber("kRateFlyWheelSpeed", kRateFlyWheelSpeed());
    SmartDashboard.putNumber("kSetPointPID", flyWheelSpeed.getSetpoint());
    SmartDashboard.putNumber("getPositionError", flyWheelSpeed.getPositionError());

    SmartDashboard.putNumber("motorOutputA", flyWheelA.get());
    SmartDashboard.putNumber("motorOutputB", flyWheelB.get());

    SmartDashboard.putBoolean("iseadytoshoot", flyWheelSpeed.atSetpoint());
    SmartDashboard.putNumber("calculateSpeedToFlyWheel", calculateSpeedToFlyWheel(Robot.distanceFromTargetLimelightY));

  }

  public double getPositionError() {
    return flyWheelSpeed.getPositionError();
  }

  /**
   * Set the power to the fly wheel motors
   * 
   * @param speed The given power
   */
  public void controlFlyWheelMotor(double speed) {
    flyWheelA.setVoltage(speed);
  }

  public void setP(double kp) {
    flyWheelSpeed.setP(kp);
  }

  /**
   * Set the angle velocity for the fly wheel
   * 
   * @return The kRate calculation
   */
  public double kRateFlyWheelSpeed() {
    return flyWheelEncoderA.getVelocity();
  }

  public void setsetPoint(double setpoint) {
    flyWheelSpeed.setSetpoint(setpoint);
  }

  /**
   * Calculate the PID for the fly wheel
   * 
   * @param setPoint The destanation the robot need to reach
   * @return The result of the calculation
   */
  public double flyWheelSpeedOutPut() {
    double kf = (12 / Chassis.RPM) * flyWheelSpeed.getSetpoint();
    return MathUtil.clamp(flyWheelSpeed.calculate(kRateFlyWheelSpeed()) + kf, 0, 12);
  }

  /**
   * @param deltaDistance The distance the robot need to reach
   * @return The result of the calculation
   */
  public double calculateSpeedToFlyWheel(double deltaDistance) {

    return (-0.00212638 * Math.pow(deltaDistance, 2)) + (3.14689 * deltaDistance) + 2061.23;

  }

  /**
   * Check if the fly wheel is on the setpoint
   * 
   * @return True if on target, false if isn't
   */
  public boolean isFlyWheelOnTraget() {
    return flyWheelSpeed.atSetpoint();
  }

  public double getRate() {
    return flyWheelEncoderA.getVelocity();
  }

  public void resetPID() {
    flyWheelSpeed.reset();
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