/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
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
  private Encoder encoderElevator;

  private PIDController elevatorPID; // PID controler for hight
  private DoubleSolenoid elevatorPiston;

  private CANDigitalInput limitSwitchUp;
  private CANDigitalInput limitSwitchDown;

  private static Elevator elevator;

  private Elevator() {
    // Motor
    elevatorMotor = new CANSparkMax(ConstantsElevator.ELEVATOR_MOTOR,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    // Encoder
    encoderElevator = new Encoder(ConstantsElevator.ELEVATOR_ENCODER_A, ConstantsElevator.ELEVATOR_ENCODER_B);
    encoderElevator.setDistancePerPulse(1);

    // PID
    elevatorPID = new PIDController(KP_ELEVATOR, KI_ELEVATOR, KD_ELEVATOR);
    elevatorPID.setTolerance(10); // TODO

    // Piston
    elevatorPiston = new DoubleSolenoid(ConstantsElevator.ELEVATOR_DOUBLE_SOLENOID_A,
        ConstantsElevator.ELEVATOR_DOUBLE_SOLENOID_B);

    // Limit Switches

    limitSwitchUp = new CANDigitalInput(elevatorMotor, LimitSwitch.kForward, LimitSwitchPolarity.kNormallyOpen);
    limitSwitchDown = new CANDigitalInput(elevatorMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
  }

  // updat the value in the smart dash bord
  public void value() {
    SmartDashboard.putNumber("encoderValue", encoderElevator.getDistance());
    SmartDashboard.putBoolean("elevatorPistonStatus", isPistonOpen());

    SmartDashboard.putBoolean("isUpLimitSwitchPressed", isUpLimitSwitchPressed());
    SmartDashboard.putBoolean("isDownLimitSwitchPressed", isDownLimitSwitchPressed());
  }

  // pid reset
  public void elevatorEncoderReset() {
    encoderElevator.reset();

  }

  public double getElevatorPIDOutput(double setpoint) {
    return MathUtil.clamp(elevatorPID.calculate(encoderElevator.getDistance(), setpoint), -1, 1);
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

  public boolean isUpLimitSwitchPressed() {
    return limitSwitchUp.get();
  }

  public boolean isDownLimitSwitchPressed() {
    return limitSwitchDown.get();
  }

  public static Elevator getinstance() {
    if (elevator == null) {
      elevator = new Elevator();
    }
    return elevator;
  }

  @Override
  public void periodic() {
    if (isDownLimitSwitchPressed()) {
      elevatorEncoderReset();
    }
    value();

  }
}