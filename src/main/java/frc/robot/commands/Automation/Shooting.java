/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Chassis.PIDVision;
import frc.robot.commands.Shooter.PIDFlyWheel;
import frc.robot.commands.ShooterTransportation.PIDSquishMotor;
import frc.robot.commands.Transportation.TransportationContorl;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransportation;
import frc.robot.subsystems.Transportation;

public class Shooting extends CommandBase {
  /**
   * Creates a new SchoolShooting.
   */

  private Automation auto;
  private CommandBase squishSpeed, transportation, flyWheel , PIDVision;

  public Shooting(Automation automation) {
    squishSpeed = new PIDSquishMotor(ShooterTransportation.getinstance());
    transportation = new TransportationContorl(Transportation.getinstance());
    flyWheel = new PIDFlyWheel(Shooter.getinstance());
    PIDVision = new PIDVision(0, 0.1, Chassis.getinstance());

    auto = automation;
    addRequirements(auto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flyWheel.schedule();
    PIDVision.schedule();
    squishSpeed.schedule();
    transportation.schedule();
    flyWheel.initialize();
    PIDVision.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    {
      
      flyWheel.execute();
      PIDVision.execute();
      
      if (Shooter.getinstance().isFlyWheelOnTraget() && PIDVision.isFinished()) {
        squishSpeed.execute();
        transportation.execute();

      } else {

        transportation.schedule();
        squishSpeed.schedule();

        transportation.cancel();
        squishSpeed.cancel();
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transportation.schedule();
    squishSpeed.schedule();
    PIDVision.schedule();
    squishSpeed.cancel();
    PIDVision.cancel();
    transportation.cancel();
    flyWheel.cancel();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
