/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;
public class roulettePath extends SequentialCommandGroup {
  /**
   * Creates a new leftRocketPath.
   */
  Chassis ch1;
  public roulettePath(Chassis ch) {
ch1 = ch;
   
    
    addCommands(
      new MAPath(0.1 , ch1)
    
    );

}
}
