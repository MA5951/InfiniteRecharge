/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;

public class PIDFlyWheelAutonumos extends CommandBase {
  /**
   * Creates a new PIDFlyWheel.
   */
  private double flyWheelSpeed;
  private Shooter shooter;
  private double kf;
  double speed;

  public PIDFlyWheelAutonumos(Shooter shooter , double speed) {
    this.speed  = speed;
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flyWheelSpeed = speed;
    kf = (1 / 250) * flyWheelSpeed;
  }
  @Override
  public void execute() {
    double power = shooter.flyWheelSpeedOutPut(flyWheelSpeed, kf);
    this.shooter.controlFlyWheelMotor(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.shooter.controlFlyWheelMotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
