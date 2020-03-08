/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Limlight;

public class PIDVision extends CommandBase {
  private Limlight limlight;
  private Chassis chassis;
  private double angle;
  private double lastTimeOnTarget;
  private double waitTime;

  public PIDVision(double angle, double waitTime, Limlight lm) {
    limlight = lm;
    this.angle = angle;
    this.waitTime = waitTime;
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    chassis.rampRate(0);
    chassis.setidilmodeBrake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.PIDvision(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    if (interrupted) {
      chassis.tankDrive(0, 0);
      chassis.reset();
      chassis.setidilmodeBrake();
    } else {
      chassis.tankDrive(0, 0);
      chassis.reset();
      chassis.setidilmodeBrake();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * if (!chassis.isPIDVisionOnTarget()) { lastTimeOnTarget =
     * Timer.getFPGATimestamp(); } return chassis.isPIDVisionOnTarget() &&
     * Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime; }
     */
    return false;
  }
}