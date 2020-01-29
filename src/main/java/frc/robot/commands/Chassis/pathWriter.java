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

public class pathWriter extends CommandBase {
  Chassis chassis;
  double dist;
  double angle;
  double time;
  double lastTime;
  double waitTime;
  double check = 0.25;
  
  public pathWriter(double check , Chassis ch) {
    this.check = check;
    chassis = ch;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    chassis.resetValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dist = chassis.average() / chassis.ticksPerMeter;
    angle = chassis.fixedAngle();
    time = Timer.getFPGATimestamp();

    if (time > waitTime) {
      System.out.printf("new double[] { %.3f, %.3f, 0.3, 10, 0.35, 0.7  },\n", dist, angle);
      waitTime += this.check;
    } else {
      lastTime = Timer.getFPGATimestamp();
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