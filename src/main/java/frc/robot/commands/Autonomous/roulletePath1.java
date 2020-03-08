/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Automation.IntakeAutomation;
import frc.robot.commands.Automation.Shooting;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.commands.Shooter.PIDFlyWheelAutonumos;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class roulletePath1 extends CommandBase {
  /**
   * Creates a new RoulletePath.
   */
  Autonomous autonomous;
  int stage = 0;
  double lastTimeOnTarget;
  CommandBase MApath, intake, shooting, shooting1, MApath1;

  public roulletePath1(Autonomous autonomous) {

    this.autonomous = autonomous;
    MApath = new MAPath(0.1, Chassis.getinstance());
    intake = new IntakeAutomation(Automation.getinstance());
    shooting = new Shooting(Automation.getinstance(), false);
    shooting1 = new Shooting(Automation.getinstance(), true);
    MApath1 = new MAPath(0.1, Chassis.getinstance());

    addRequirements(autonomous);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    shooting.initialize();
    MApath.initialize();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (stage) {
    case 0:
      MApath.execute();
      if (MApath.isFinished()) {
        stage++;
        MApath.end(true);
        MAPath.pathnum = 1;
        lastTimeOnTarget = Timer.getFPGATimestamp();
      }
      break;
    case 1:

      if (Timer.getFPGATimestamp() - lastTimeOnTarget < 3.8) {
        shooting.execute();
      } else {
        shooting.end(true);
        MApath1.initialize();
        stage++;
      }
      break;

    case 2:
      MApath1.execute();
      if (MAPath.stage > 2) {
        intake.initialize();
        intake.execute();

      } else if (MAPath.stage == 4) {
        intake.end(true);
        Intake.getinstance().intakeSolenoidControl(Value.kReverse);

      }

      if (MApath.isFinished()) {
        stage++;
      }
      break;

    case 3:
      MApath.end(true);
      // preshooting.end(true);
      stage++;
      break;
    case 4:

      shooting1.initialize();
      stage++;
      break;
    case 5:
      Intake.getinstance().intakeMotorControl(-0.2);
      shooting1.execute();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.getinstance().intakeMotorControl(0);
    shooting.end(true);
    intake.end(true);
    MApath.end(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
