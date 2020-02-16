/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Roulette.roundTwoRoulettePID;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Roulette;

public class RouletteAutomation extends CommandBase {
  /**
   * Creates a new Roulette.
   */
  Automation auto;
  CommandBase roulettestageone, roulettestagetwo;
  int stage;

  public RouletteAutomation(Automation auto) {
    this.auto = auto;
    roulettestageone = new roundTwoRoulettePID(0.1, Roulette.getinstance());
    roulettestagetwo = new roundTwoRoulettePID(0.1, Roulette.getinstance());
    stage = 0;
    addRequirements(auto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Roulette.getinstance().controlroulettSolenoid(true);

    if (roulettestageone.isFinished()) {
      stage++;
    }

    if (stage == 1) {
      roulettestagetwo.schedule();
      roulettestagetwo.initialize();
    } else {
      roulettestageone.schedule();
      roulettestageone.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stage == 1) {
      roulettestagetwo.execute();
    } else {
      roulettestageone.execute();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Roulette.getinstance().controlroulettSolenoid(false);
    roulettestageone.schedule();
    roulettestagetwo.schedule();
    roulettestageone.cancel();
    roulettestagetwo.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stage == 0) {
      return roulettestageone.isFinished();
    } else {
      return roulettestagetwo.isFinished();
    }
  }
}
