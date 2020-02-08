/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.Chassis.PIDVision;
import frc.robot.commands.Shooter.PIDFlyWheel;
import frc.robot.commands.Transportation.TransportationContorl;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransportation;
import frc.robot.subsystems.Transportation;

public class PreparationShooting extends CommandBase {
  /**
   * Creates a new PreSchoolShooting.
   */

  private Automation auto;

  private CommandBase visionPID, flyWheelPID, transportation, changeShooterAngle;

  public PreparationShooting(Automation automation) {
    visionPID = new PIDVision(0, 0.1, Chassis.getinstance());
    flyWheelPID = new PIDFlyWheel(Shooter.getinstance());
    transportation = new TransportationContorl(Transportation.getinstance());

    auto = automation;
    addRequirements(auto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flyWheelPID.schedule();
    Robot.isShootingPrepared = false;
    visionPID.initialize();
    flyWheelPID.initialize();
    if (Chassis.getinstance().distance() > 1) { // TODO Check the real ditance
      changeShooterAngle.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionPID.execute();
    flyWheelPID.execute();

    if (!ShooterTransportation.getinstance().isBallInShooter()) {
      transportation.execute();
    } else {
      transportation.cancel();
    }

    if (Shooter.getinstance().isFlyWheelOnTraget() && Chassis.getinstance().isPIDVisionOnTarget()) {
      Robot.isShootingPrepared = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionPID.cancel();
    flyWheelPID.cancel();
    transportation.cancel();
    changeShooterAngle.cancel();
    Robot.isShootingPrepared = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
