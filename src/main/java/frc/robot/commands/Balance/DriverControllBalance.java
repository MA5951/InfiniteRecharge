/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Balance;

public class DriverControllBalance extends CommandBase {
  /**
   * Creates a new DriverControllBalance.
   */
  Balance balance;

  public DriverControllBalance(Balance bl) {
    // Use addRequirements() here to declare subsystem dependencies.
    balance = bl;
    addRequirements(balance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      balance.controlBalanceMotor(RobotContainer.OperatingJoystick.getRawAxis(4) * 0);
    
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