/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class PIDVision extends CommandBase {
  private Chassis chassis;
  private double angle;
  private double lastTimeOnTarget;
  private double waitTime;

  
  public PIDVision( double angle, double waitTime , Chassis ch) {
 
    this.angle = angle;
    this.waitTime = waitTime;
    chassis = ch;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.rampRate(0);
    chassis.setidilmodeBrake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.PIDvision(angle) ;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      chassis.tankDrive(0, 0);
      chassis.reset();
      chassis.setidilmodeBrake();
    }
    else {
      chassis.tankDrive(0, 0);
      chassis.reset();
      chassis.setidilmodeBrake();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!chassis.isPIDVisionOnTarget()) {
      lastTimeOnTarget = Timer.getFPGATimestamp();
    }
    return chassis.isPIDVisionOnTarget() && Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime;
  }
}