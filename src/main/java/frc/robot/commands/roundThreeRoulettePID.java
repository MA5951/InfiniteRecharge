/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Roulette;

public class roundThreeRoulettePID extends CommandBase {
  /**
   * The PID command for round 3.
   */

  private Roulette roulette;
  private double speed;
  private double lastTimeOnTarget;
  private double waitTime;
  private int roundThreeColorSetpoint;
  private String gameData = DriverStation.getInstance().getGameSpecificMessage();
   
  public roundThreeRoulettePID(double waitTime, Roulette roulette) {
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
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          // Blue case code
          speed = roulette.spinPidOutput(roulette.blue());
          break;
        case 'G' :
          // Green case code
          speed = roulette.spinPidOutput(roulette.green());
          break;
        case 'R' :
          //red case code
          speed = roulette.spinPidOutput(roulette.red());
          break;
        case 'Y' :
          //Yellow case code
          speed = roulette.spinPidOutput(roulette.yellow());
          break;
        default :
          //This is corrupt data
          roulette.controlSpeed(0);
          break;
      }
    } else {
      //Code for no data received yet
      roulette.controlSpeed(0);
    }
    // Set the PID for the given color
    roulette.controlSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      roulette.controlSpeed(0);
    } else {
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
