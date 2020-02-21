/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Balance;
import frc.robot.subsystems.Elevator;

public class ElevatorMotorControl extends CommandBase {
  private Elevator elevator;

  public ElevatorMotorControl(Elevator el) {
    elevator = el;
    addRequirements(elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.OperatingJoystick.getRawButton(9)){
      elevator.setElvatorMotorSpeed(RobotContainer.OperatingJoystick.getRawAxis(5));
    }else{
      elevator.setElvatorMotorSpeed(0);
    }
     
    }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setElvatorMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}