/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Transportation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.ShooterTransportation;
import frc.robot.subsystems.Transportation;

public class pushTransportationAndSquish extends CommandBase {
  /**
   * Creates a new pushTransportationAndSquish.
   */
  Transportation transportation;
  Automation auto;
  double speed;
  ShooterTransportation shooterTransportation;
  public pushTransportationAndSquish(Automation auto) {
    this.auto = auto;
    transportation = Transportation.getinstance();
    shooterTransportation = ShooterTransportation.getinstance();
    addRequirements(auto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transportation.transportationControl(0.5);
    shooterTransportation.controlSquishMotor(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterTransportation.controlSquishMotor(0);
    transportation.transportationControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
