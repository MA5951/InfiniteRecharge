/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Chassis;

public class LimelightAngle3DToZeroPID extends CommandBase {
  /**
   * Creates a new limelightAngle3DPID.
   */
  Chassis chassis;
  double setPoint;

  public LimelightAngle3DToZeroPID(Chassis ch , double setPoint) {
    this.setPoint = setPoint;
    // Use addRequirements() here to declare subsystem dependencies.
    chassis = ch;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.tankDrive(-chassis.angleThreeDLimelightPIDOutput(setPoint), chassis.angleThreeDLimelightPIDOutput(setPoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.isLimeLightOnTarget();
  }
}
