/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Automation.IntakeAutomation;
import frc.robot.commands.Automation.Shooting;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Chassis;

public class RoulletePath extends CommandBase {
  /**
   * Creates a new RoulletePath.
   */
  Autonomous autonomous;
  int stage = 0;
  CommandBase MApath, Intake, shooting;

  public RoulletePath(Autonomous autonomous) {

    this.autonomous = autonomous;
    MApath = new MAPath(0.1, Chassis.getinstance());
    Intake = new IntakeAutomation(Automation.getinstance());
    shooting = new Shooting(Automation.getinstance());

    addRequirements(autonomous);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    MApath.schedule();
    Intake.schedule();
    shooting.schedule();

    shooting.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (stage) {
    case 0:
      if (Timer.getFPGATimestamp() < 1) {
        shooting.execute();

      } else {
        stage++;
        shooting.cancel();
        MApath.initialize();
      }
      break;

    case 1:
      MApath.execute();
      if (MAPath.stage == 2) {
        Intake.initialize();
      } else if (MAPath.stage > 3) {
        Intake.execute();
      }

      if (MApath.isFinished()) {
        stage++;
      }
      break;

    case 2:
      MApath.cancel();
      Intake.cancel();
      stage++;
      break;
    case 3:
      shooting.schedule();
      shooting.initialize();
      stage++;
      break;
    case 4:
      shooting.execute();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooting.schedule();
    Intake.schedule();
    MApath.schedule();
    shooting.cancel();
    Intake.cancel();
    MApath.cancel();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
