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
    double leftPower = chassis.leftvelocityControl(RobotContainer.leftJoystick.getY() * chassis.RPM);
    double rightPower = chassis.rightvelocityControl(RobotContainer.rightJoystick.getY() * chassis.RPM);

    if (RobotContainer.rightJoystick.getY() > 0.1 || RobotContainer.rightJoystick.getY() < -0.1) {
      if (RobotContainer.rightJoystick.getRawButton(1) || RobotContainer.leftJoystick.getRawButton(1)) {
        rightPower = chassis.rightvelocityControl(RobotContainer.rightJoystick.getY() * chassis.RPM * 0.3);
        chassis.rightcontrol(rightPower * 0.3);
      } else {
        chassis.rightcontrol(rightPower);
      }
    } else {
      rightPower = chassis.rightvelocityControl(0);

      chassis.rightcontrol(rightPower);
    }

    if (RobotContainer.leftJoystick.getY() > 0.1 || RobotContainer.leftJoystick.getY() < -0.1) {

      if (RobotContainer.rightJoystick.getRawButton(1) || RobotContainer.leftJoystick.getRawButton(1)) {
        leftPower = chassis.leftvelocityControl(RobotContainer.leftJoystick.getY() * chassis.RPM * 0.3);
        chassis.leftcontrol(leftPower * 0.3);
      } else {
        chassis.leftcontrol(leftPower);
      }
    } else {
      leftPower = chassis.leftvelocityControl(0);
      chassis.leftcontrol(leftPower);
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
