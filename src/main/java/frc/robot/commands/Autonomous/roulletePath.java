/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Automation.IntakeAutomation;
import frc.robot.commands.Automation.Shooting;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;

public class roulletePath extends CommandBase {
  /**
   * Creates a new roulletePath.
   */
  private CommandBase intakeCommand, Shooting, maPath;

  private Autonomous auto;
  private int stage;
  private int shootCounter;

  public roulletePath(Autonomous autonomous, int stage, int shootCounter) {
    this.shootCounter = shootCounter;
    Shooting = new Shooting(Automation.getinstance());
    intakeCommand = new IntakeAutomation(Automation.getinstance());
    maPath = new MAPath(0.1, Chassis.getinstance());
    this.stage = stage;
    auto = autonomous;
    addRequirements(auto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeCommand.schedule();
    Shooting.schedule();
    
    Shooting.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Shooter.getinstance().shootCounter < shootCounter) {
      Shooting.execute();
    } else {
      Shooting.cancel();
      if (MAPath.stage > stage) {
        intakeCommand.schedule();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeCommand.cancel();
    Shooting.cancel();
    maPath.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return maPath.isFinished();
  }
}
