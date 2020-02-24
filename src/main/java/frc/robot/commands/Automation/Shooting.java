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
import frc.robot.subsystems.Limlight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransportation;
import frc.robot.subsystems.Transportation;

public class Shooting extends CommandBase {
  /**
   * Creates a new SchoolShooting.
   */

  private Automation auto;
  private CommandBase squishSpeed, transportation, flyWheel , PIDVision;
  private boolean drive;

  public Shooting(Automation automation, boolean drive) {
    this.drive = drive;
    squishSpeed = new PIDSquishMotor(ShooterTransportation.getinstance());
    transportation = new TransportationContorl(Transportation.getinstance());
    flyWheel = new PIDFlyWheel(Shooter.getinstance());
    PIDVision = new PIDVision(0, 0.1, Limlight.getInstance());

    auto = automation;
    addRequirements(auto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if (drive) {
      
      PIDVision.initialize();
    }
    
    flyWheel.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    {
      
      flyWheel.execute();
      if (drive) {
      PIDVision.execute();
      }
      
      if (Shooter.getinstance().isFlyWheelOnTraget() && Chassis.getinstance().isPIDVisionOnTarget()) {
        squishSpeed.execute();
        transportation.execute();
        PIDVision.end(true);

      } else {

       

        transportation.end(true);
        squishSpeed.end(true);
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (drive) {
      PIDVision.end(true);
    }
 
    squishSpeed.end(true);
    transportation.end(true);
    flyWheel.end(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
