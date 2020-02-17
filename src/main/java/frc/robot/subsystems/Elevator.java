/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ConstantsElevator;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Elevator extends SubsystemBase {

  private static final double KP_ELEVATOR = 0;
  private static final double KI_ELEVATOR = 0;
  private static final double KD_ELEVATOR = 0;

  private CANSparkMax elevatorMotor;


  private PIDController elevatorPID; // PID controler for hight
  private DoubleSolenoid elevatorPiston;

  private DigitalInput elevatorLimitSwich;
  private CANEncoder canEncoder;

  private static Elevator elevator;

  private Elevator() {
    // Motor
    elevatorMotor = new CANSparkMax(ConstantsElevator.ELEVATOR_MOTOR,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    // Encoder


    // PID
    elevatorPID = new PIDController(KP_ELEVATOR, KI_ELEVATOR, KD_ELEVATOR);
    elevatorPID.setTolerance(10); // TODO

    // Piston
    elevatorPiston = new DoubleSolenoid(ConstantsElevator.ELEVATOR_DOUBLE_SOLENOID_A,
        ConstantsElevator.ELEVATOR_DOUBLE_SOLENOID_B);

    // Limit Switches
    canEncoder = elevatorMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 1);
    canEncoder.setPositionConversionFactor(1);
    elevatorLimitSwich = new DigitalInput(1);

  }

  // updat the value in the smart dash bord
  public void value() {
    SmartDashboard.putNumber("encoderValue", canEncoder.getPosition());
    SmartDashboard.putBoolean("elevatorPistonStatus", isPistonOpen());
    SmartDashboard.putBoolean("limiswichdownelevator", elevatorLimitSwich.get());


  }

  // pid reset
  public void elevatorEncoderReset() {
    canEncoder.setPosition(0);

  }

  public double getElevatorPIDOutput(double setpoint) {
    return MathUtil.clamp(elevatorPID.calculate(canEncoder.getPosition(), setpoint), -1, 1);
  }

  public void setElvatorMotorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public boolean isPIDElevetorOnTarget() {
    return elevatorPID.atSetpoint();
  }

  public void openElevatorPiston() {
    elevatorPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void closeElevatorPiston() {
    elevatorPiston.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isPistonOpen() {
    return elevatorPiston.get() == Value.kForward;
  }

public boolean islimitswichdown(){
  return elevatorLimitSwich.get();
}

  public static Elevator getinstance() {
    if (elevator == null) {
      elevator = new Elevator();
    }
    return elevator;
  }

  @Override
  public void periodic() {
    if (elevatorLimitSwich.get()) {
      elevatorEncoderReset();
    }
  
    value();

  }
}