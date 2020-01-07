/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class leftRocketPath extends SequentialCommandGroup {
  /**
   * Creates a new leftRocketPath.
   */
  Chassis ch1;
  public leftRocketPath(Chassis ch) {
ch1 = ch;
   
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      new MAPath(0.1 , ch1)
      // new 30, 0, 5, 3, 0.1);
      // new MAPath(0.1);
      // new MAPath(0.1);
      // new PIDVision(30, 0, 5, 3, 0.1);
    );

}
}
