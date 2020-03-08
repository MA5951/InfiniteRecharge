/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Intake.IntakePullPush;
import frc.robot.commands.Intake.OpenIntake;
import frc.robot.commands.ShooterTransportation.PIDSquishMotor;
import frc.robot.commands.Transportation.TransportationContorl;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterTransportation;
import frc.robot.subsystems.Transportation;

public class IntakeAutomation extends CommandBase {
  /**
   * Creates a new IntakeAutomation.
   */

  private Automation auto;

  private CommandBase piston;
  private CommandBase roller;

  public IntakeAutomation(Automation automation) {
    // Use addRequirements() here to declare subsystem dependencies.
    piston = new OpenIntake(Intake.getinstance());
    roller = new IntakePullPush(-0.5, Intake.getinstance()); // TODO Enter real speed value
    auto = automation;
    addRequirements(Autonomous.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    roller.schedule();
 
    piston.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roller.execute();
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roller.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
