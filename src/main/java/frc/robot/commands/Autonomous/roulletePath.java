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
  double lastTimeOnTarget;
  CommandBase MApath, Intake, shooting;

  public RoulletePath(Autonomous autonomous) {

    this.autonomous = autonomous;
    MApath = new MAPath(0.1, Chassis.getinstance());
    Intake = new IntakeAutomation(Automation.getinstance());
    shooting = new Shooting(Automation.getinstance(), false);

    addRequirements(autonomous);
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
    // System.out.println(stage);
    switch (stage) {
    case 0:

      if (Timer.getFPGATimestamp() - lastTimeOnTarget < 10) {
        shooting.execute();
      } else {
        shooting.end(true);
        MApath.initialize();
        stage++;
      }
      break;

    case 1:
      MApath.execute();
      if (MAPath.stage == 3) {
        Intake.initialize();
      } else if (MAPath.stage == 4){
        Intake.execute();
      }else if(MAPath.stage == 5){
        Intake.end(true);
      }
   
        if (MApath.isFinished()) {
          stage++;
        }
      break;

    case 2:
      MApath.end(true);
     
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

    shooting.end(true);
    Intake.end(true);
    MApath.end(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
