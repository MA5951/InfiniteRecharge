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
import frc.robot.commands.Shooter.PIDFlyWheelAutonumos;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;

public class EnemyRoullete extends CommandBase {
  /**
   * Creates a new RoulletePath.
   */
  Autonomous autonomous;
  int stage = 0;
  double lastTimeOnTarget;
  CommandBase MApath, Intake, shooting , preshooting;

  public EnemyRoullete(Autonomous autonomous) {

    this.autonomous = autonomous;
    MApath = new MAPath(0.1, Chassis.getinstance());
    Intake = new IntakeAutomation(Automation.getinstance());
    shooting = new Shooting(Automation.getinstance(), true);
    preshooting = new PIDFlyWheelAutonumos(Shooter.getinstance() , 190);

    addRequirements(autonomous);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    preshooting.initialize();
    Intake.initialize();
    MApath.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(stage);
    switch (stage) {
    case 0:
        Intake.execute();
        MApath.execute();
        preshooting.execute();
        if(MAPath.stage == 2){
          Intake.end(true);
        }

        if(MApath.isFinished()){
          MApath.end(true);
          preshooting.end(true);
          shooting.initialize();
          stage++;
        }
        
      break;
      
    case 1:

      shooting.execute();
      break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooting.end(true);
    Intake.end(true);
    MApath.end(true);
    preshooting.end(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
