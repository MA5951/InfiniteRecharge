/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Roulette;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransportation;
import frc.robot.subsystems.Transportation;

public class CancelAllMotors extends InstantCommand {
  Automation auot;

  public CancelAllMotors(Automation auot) {
    this.auot = auot;
    addRequirements(auot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Intake.getinstance().intakeMotorControl(0);
    Transportation.getinstance().transportationControl(0);
    Roulette.getinstance().controlroulettSolenoid(false);
    Shooter.getinstance().controlFlyWheelMotor(0);
    ShooterTransportation.getinstance().controlSquishMotor(0);
  }
}
