/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Intake.IntakeOpenClose;
import frc.robot.commands.Intake.IntakePullPush;
import frc.robot.commands.ShooterTransportation.PIDSquishMotor;
import frc.robot.commands.Transportation.TransportationContorl;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransportation;
import frc.robot.subsystems.Transportation;

public class IntakeAutomation extends CommandBase {
  /**
   * Creates a new IntakeAutomation.
   */

  private Automation auto;

  private CommandBase piston;
  private CommandBase roller;
  private CommandBase transportation;
  private CommandBase PIDSquish;

  public IntakeAutomation(Automation automation) {
    // Use addRequirements() here to declare subsystem dependencies.
    piston = new IntakeOpenClose(Intake.getinstance());
    roller = new IntakePullPush(0.7, Intake.getinstance()); // TODO Enter real speed value
    transportation = new TransportationContorl(Transportation.getinstance());
    PIDSquish = new PIDSquishMotor(ShooterTransportation.getinstance());
    auto = automation;
    addRequirements(auto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    piston.schedule();
    roller.schedule();
    transportation.schedule();
    PIDSquish.schedule();
    
    piston.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ShooterTransportation.getinstance().isBallInShooter()) {
      PIDSquish.execute();
    } else {
      PIDSquish.cancel();
    }
    roller.execute();
    transportation.execute();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roller.cancel();
    transportation.cancel();
    PIDSquish.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
