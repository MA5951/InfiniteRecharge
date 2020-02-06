/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
    chassis.resetVelocityControl();
    chassis.rampRate(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //  if(Math.abs(RobotContainer.leftJoystick.getY()) <  0.1 &&  Math.abs(RobotContainer.rightJoystick.getY()) <  0.1){
    //    chassis.tankDrive(0, 0);
    //        } else if(RobotContainer.leftJoystick.getRawButton(1) || RobotContainer.rightJoystick.getRawButton(1) ){
    //          chassis.tankDrive(chassis.leftVelocityControlSetPoint(RobotContainer.leftJoystick.getY() *(chassis.RPM /4)),
    //              chassis.rightVelocityControlSetPoint(RobotContainer.rightJoystick.getY() *(chassis.RPM / 4) ));
    //        }else{
    //         chassis.tankDrive(chassis.leftVelocityControlSetPoint(RobotContainer.leftJoystick.getY() * chassis.RPM),
    //       chassis.rightVelocityControlSetPoint(RobotContainer.rightJoystick.getY() *chassis.RPM) );
    //       }
          double left = RobotContainer.leftJoystick.getY();
          double right = RobotContainer.rightJoystick.getY();
          if(left<0.1&&left>-0.1){
            left = 0;
          }
          else if(right <0.1&&right >-0.1){
            right = 0;
          }
          chassis.tankDrive(left,right);

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
