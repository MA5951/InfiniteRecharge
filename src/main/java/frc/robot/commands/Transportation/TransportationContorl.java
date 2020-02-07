/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Transportation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Transportation;

public class TransportationContorl extends CommandBase {
  /**
   * Creates a new TransportationContorl.
   */
  private Transportation transportation;
  public TransportationContorl(Transportation tr) {
    // Use addRequirements() here to declare subsystem dependencies.
    transportation = tr;
    addRequirements(transportation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transportation.transportationControl(0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transportation.transportationControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
