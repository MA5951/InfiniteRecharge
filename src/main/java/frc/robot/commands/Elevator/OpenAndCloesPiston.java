/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class OpenAndCloesPiston extends InstantCommand {
  Elevator el; 
  boolean closeOrOpen = false;
  public OpenAndCloesPiston(Elevator elvetor) {
    el = elvetor;
   addRequirements(el);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    closeOrOpen =!closeOrOpen;
    if(closeOrOpen){
      el.openElevatorPiston();
    }else{
      el.closeElevatorPiston();
    }


  }
}
