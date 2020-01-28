/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private static Intake intake;

  private DoubleSolenoid intakeSolenoid;

  private WPI_VictorSPX intakeVictorSPX;

  public Intake() {
      intakeSolenoid = new DoubleSolenoid(Constants.IntakeSolenoidOne, Constants.IntakeSolenoidTwo);

      intakeVictorSPX = new WPI_VictorSPX(Constants.IntakeMotor);
  }

  public void intakeSolenoidControl(Value value) {
    intakeSolenoid.set(value);
  }

  public void intakeMotorControl(double power) {
    intakeVictorSPX.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public static Intake getinstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }
}