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
  Elevator el; 
  public OpenAndClosePiston(Elevator elvetor) {
    el = elvetor;
   addRequirements(el);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(el.isPistonOpen()){
      el.openElevatorPiston();
    }else{
      el.closeElevatorPiston();
    }


  }
}
