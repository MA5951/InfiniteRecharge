/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

public class tankDrive extends CommandBase {
  Chassis chassis;

  public tankDrive(Chassis ch) {
    chassis = ch;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.rampRate(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.rightJoystick.getY() > 0.1 || RobotContainer.rightJoystick.getY() < -0.1) {
      if (RobotContainer.rightJoystick.getRawButton(1) || RobotContainer.leftJoystick.getRawButton(1)) {

        chassis.rightcontrol(RobotContainer.rightJoystick.getY() * 0.3 * 12);
      } else {
        chassis.rightcontrol(RobotContainer.rightJoystick.getY() * 12);
      }
    } else {

      chassis.rightcontrol(0);
    }

    if (RobotContainer.leftJoystick.getY() > 0.1 || RobotContainer.leftJoystick.getY() < -0.1) {

      if (RobotContainer.rightJoystick.getRawButton(1) || RobotContainer.leftJoystick.getRawButton(1)) {

        chassis.leftcontrol(RobotContainer.leftJoystick.getY() * 0.3 * 12);
      } else {
        chassis.leftcontrol(RobotContainer.leftJoystick.getY() * 12);
      }
    } else {

      chassis.leftcontrol(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
