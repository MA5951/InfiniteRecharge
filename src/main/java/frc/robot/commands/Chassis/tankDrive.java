/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.RobotContainer;
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
    chassis.resetVelocityControl();
    chassis.rampRate(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
            //if(RobotContainer.leftJoystick.getRawButton(1) || RobotContainer.rightJoystick.getRawButton(1) ){
            //chassis.tankDrive(chassis.leftVelocityControlSetPoint(RobotContainer.leftJoystick.getY() *(chassis.RPM /4)),
             //    chassis.rightVelocityControlSetPoint(RobotContainer.rightJoystick.getY() *(chassis.RPM / 4)));
           //}else{
           //chassis.tankDrive(chassis.leftVelocityControlSetPoint(RobotContainer.leftJoystick.getY() * chassis.RPM),
         // chassis.rightVelocityControlSetPoint(RobotContainer.rightJoystick.getY() *chassis.RPM) );
       // }
       chassis.getinstance().tankDrive(RobotContainer.leftJoystick.getY(), RobotContainer.rightJoystick.getY());
 
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
