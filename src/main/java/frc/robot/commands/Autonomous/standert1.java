/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Automation.Shooting;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Chassis;

public class standert1  extends CommandBase {
  /**
   * Creates a new shootanddrive.
   */
  double lastTimeOnTarget;
  CommandBase MApath, shooting;
  int stage = 0;

  public standert1() {
    addRequirements(Autonomous.getInstance());
    MApath = new MAPath(0.1, Chassis.getinstance());
    shooting = new Shooting(Automation.getinstance(), false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    shooting.initialize();
    lastTimeOnTarget = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (stage) {
    case 0:

    if (Timer.getFPGATimestamp() - lastTimeOnTarget < 6.5) {
      shooting.execute();
    } else {
      shooting.end(true);
      MApath.initialize();
      stage++;
    }

      break;

    case 1:
    MApath.execute();

        break;
      }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooting.end(true);
    MApath.end(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
