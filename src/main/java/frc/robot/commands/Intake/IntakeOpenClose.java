/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeOpenClose extends InstantCommand {
  private Intake intake;

  public IntakeOpenClose(Intake in) {
    intake = in;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intake.isPistonOpen()) {
      intake.intakeSolenoidControl(Value.kReverse);
    } else {
      intake.intakeSolenoidControl(Value.kForward);
    }
  }
}
