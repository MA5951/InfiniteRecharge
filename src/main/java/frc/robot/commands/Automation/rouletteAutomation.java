/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Elevator.ElevetorPID;
import frc.robot.commands.Roulette.roundThreeRoulettePID;
import frc.robot.commands.Roulette.roundTwoRoulettePID;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Roulette;

public class rouletteAutomation extends CommandBase {
  /**
   * Creates a new rouletteAutomation.
   */

  private Automation auto;
  private CommandBase elevatorPID, roulletePIDRoundTwo, roulettePIDRoundThree;
  private boolean round;

  public rouletteAutomation(Automation automation) {
    elevatorPID = new ElevetorPID(Elevator.getinstance(), 1, 0.1); // TODO Check real distance
    roulletePIDRoundTwo = new roundTwoRoulettePID(0.1, Roulette.getinstance());
    roulettePIDRoundThree = new roundThreeRoulettePID(0.1, Roulette.getinstance());
    round = false;

    auto = automation;
    addRequirements(auto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorPID.schedule();
    roulettePIDRoundThree.schedule();
    roulletePIDRoundTwo.schedule();
    
    if (roulletePIDRoundTwo.isFinished()) {
      round = true;
    }
    if (!round) {
      roulletePIDRoundTwo.initialize();
    } else {
      roulettePIDRoundThree.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!elevatorPID.isFinished()) {
      elevatorPID.execute();
    } else {
      elevatorPID.cancel();
      if (round) {
        roulettePIDRoundThree.execute();
      } else {
        roulletePIDRoundTwo.execute();
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roulletePIDRoundTwo.cancel();
    roulettePIDRoundThree.cancel();
    elevatorPID.cancel();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
