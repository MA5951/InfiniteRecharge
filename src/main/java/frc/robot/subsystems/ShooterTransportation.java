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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.*;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class ShooterTransportation extends SubsystemBase {
  private static ShooterTransportation shooterTransportation;

  private TalonSRX squishMotor;

  private DigitalInput IRBall;

  private edu.wpi.first.wpilibj.controller.PIDController squishMotorSpeed;

  private ShooterTransportation() {

    squishMotor = new TalonSRX(ShooterConstants.SQUISH_MOTOR);
    IRBall = new DigitalInput(ShooterConstants.IR_BALL);
  }

  /**
   * Display The given values to the shuffleboard
   */
  public void ShooterValue() {
    SmartDashboard.putBoolean("IRball", !IRBall.get());
    SmartDashboard.putNumber("kSetPointPIDSquish", squishMotorSpeed.getSetpoint());
    SmartDashboard.putNumber("squishMotorCurrent", squishMotor.getStatorCurrent());
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

  @Override
  public void periodic() {
    ShooterValue();

  }
}