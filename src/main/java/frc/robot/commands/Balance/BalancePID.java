/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Balance;

public class BalancePID extends CommandBase {
  /**
   * Creates a new BalancePID.
   */

  private Balance balance;


  public BalancePID( Balance bl) {
    // Use addRequirements() here to declare subsystem dependencies.
    balance = bl;
    addRequirements(balance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balance.changeModeSparkMaxBrake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double power = balance.balancePidControllerSetPoint(angle);
    // balance.controlBalanceMotor(power);

    double threshold = 3;
    double scale = 0.5;
    double angle = balance.getinstance().getnavxPich();
    double power = (Math.abs(angle) / angle) * scale;

    if (Math.abs(angle) > threshold) {
      balance.getinstance().controlBalanceMotor(power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    balance.controlBalanceMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}