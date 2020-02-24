/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class OpenAndClosePiston extends InstantCommand {
  private Elevator el;
  private boolean value;

  public OpenAndClosePiston(Elevator elvetor, boolean value) {
    this.value = value;
    el = elvetor;
    addRequirements(el);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    el.ControlElevatorPiston(value);
  }

}
