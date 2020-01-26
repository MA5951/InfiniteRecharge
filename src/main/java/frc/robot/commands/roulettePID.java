/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Roulette;

public class roulettePID extends CommandBase {
  /**
   * roulettePID command.
   */

  private Roulette roulette;
  private double setpoint;
  private double speed;
  private double lastTimeOnTarget;
  private double waitTime;

  public roulettePID(double setpoint, double waitTime, Roulette roulette) {
    this.setpoint = setpoint;
    this.waitTime = waitTime;
    this.roulette = roulette;
    addRequirements(roulette);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    roulette.resetTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = roulette.spinPidOutput(setpoint);
    roulette.controlSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      roulette.controlSpeed(0);
      roulette.resetTicks();
    } else {
      roulette.controlSpeed(0);
      roulette.controlSpeed(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!roulette.isPIDOnTarget()) {
      lastTimeOnTarget = Timer.getFPGATimestamp();
    }
    return roulette.isPIDOnTarget() && Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime;
  }
}
