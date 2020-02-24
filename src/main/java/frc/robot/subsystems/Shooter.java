/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

  private double KP_FLY_WHEEL_SPEED = 0.05;
  private double KI_FLY_WHEEL_SPEED = 0.00055;
  private double KD_FLY_WHEEL_SPEED = 0;
  
  private double ticksPerRoundflyWheel = 999.5; // TODO

  private TalonSRX flyWheelA;
  private TalonSRX flyWheelB;

  public double shootCounter = 0;

  private edu.wpi.first.wpilibj.controller.PIDController flyWheelSpeed;

  private Shooter() {

    flyWheelA = new TalonSRX(ShooterConstants.FLY_WHEEL_A);
    flyWheelB = new TalonSRX(ShooterConstants.FLY_WHEEL_B);
    
    flyWheelA.configPeakCurrentLimit(30);
    flyWheelB.configPeakCurrentLimit(30);
    

    flyWheelSpeed = new edu.wpi.first.wpilibj.controller.PIDController(KP_FLY_WHEEL_SPEED, KI_FLY_WHEEL_SPEED,
        KD_FLY_WHEEL_SPEED);
      
    flyWheelSpeed.setTolerance(5);
flyWheelSpeed.setIntegratorRange(-1000, 1000);
    flyWheelB.configClosedloopRamp(0.05);
    flyWheelA.configClosedloopRamp(0.05);

  }

  /**
   * Display The given values to the shuffleboard
   */
  public void ShooterValue() {
    SmartDashboard.putNumber("kRateFlyWheelSpeed", kRateFlyWheelSpeed());
    SmartDashboard.putNumber("kSetPointPID", flyWheelSpeed.getSetpoint());
    SmartDashboard.putNumber("getPositionError", flyWheelSpeed.getPositionError());

    SmartDashboard.putNumber("motorOutputA", flyWheelA.getMotorOutputPercent());
    SmartDashboard.putNumber("motorOutputB", flyWheelB.getMotorOutputPercent());

    SmartDashboard.putBoolean("iseadytoshoot", flyWheelSpeed.atSetpoint());
    SmartDashboard.putNumber("calculateSpeedToFlyWheel", calculateSpeedToFlyWheel(Robot.distanceFromTargetLimelightY));

  }

  /**
   * Set the power to the fly wheel motors
   * 
   * @param speed The given power
   */
  public void controlFlyWheelMotor(double speed) {
    System.out.println(speed);
    flyWheelB.set(ControlMode.PercentOutput, speed);
    flyWheelA.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set the angle velocity for the fly wheel
   * 
   * @return The kRate calculation
   */
  public double kRateFlyWheelSpeed() {
    return (flyWheelA.getSelectedSensorVelocity() * (2 * Math.PI)) / ticksPerRoundflyWheel;
  }

  /**
   * Calculate the PID for the fly wheel
   * 
   * @param setPoint The destanation the robot need to reach
   * @return The result of the calculation
   */
  public double flyWheelSpeedOutPut(double setPoint, double kf) {
    kf = 0;
    return MathUtil.clamp(flyWheelSpeed.calculate(kRateFlyWheelSpeed(), setPoint) + kf, 0, 0.85);
  }

  /**
   * @param deltaDistance The distance the robot need to reach
   * @return The result of the calculation
   */
  public double calculateSpeedToFlyWheel(double deltaDistance) {

    return (0.000222855* Math.pow(Chassis.getinstance().distance() , 2)) + (-0.145308 * Chassis.getinstance().distance()) + 214.5;

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
    return flyWheelA.getSelectedSensorVelocity();
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