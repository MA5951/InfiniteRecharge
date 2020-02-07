/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ShooterTransportation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransportation;

public class PIDSquishMotor extends CommandBase {
  /**
   * Creates a new PIDSquishMotor.
   */
  private ShooterTransportation shooterTransportation;
  public PIDSquishMotor(ShooterTransportation shooterTransportation) {
    this.shooterTransportation = shooterTransportation;
     addRequirements(this.shooterTransportation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = shooterTransportation.squishMotorSpeedOutput();
    this.shooterTransportation.controlSquishMotor(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

      this.shooterTransportation.controlSquishMotor(0);
    
 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}