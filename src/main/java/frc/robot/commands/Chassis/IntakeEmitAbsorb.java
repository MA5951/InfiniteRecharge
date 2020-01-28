/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeEmitAbsorb extends CommandBase {
  /**
   * Creates a new IntakeEmitAbsorb.
   */
  private  Intake intake;

  private boolean isIntakeReallyAbsorbing;
  public IntakeEmitAbsorb(Boolean isIntakeAbsorbing,  Intake in) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = in;
    isIntakeReallyAbsorbing = isIntakeAbsorbing;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isIntakeReallyAbsorbing) {
      intake.intakeMotorControl(0.5);
    } else if (!isIntakeReallyAbsorbing) {
      intake.intakeMotorControl(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    intake.intakeMotorControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
