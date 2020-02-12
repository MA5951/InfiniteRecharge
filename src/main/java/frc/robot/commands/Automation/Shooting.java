/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Shooter.PIDFlyWheel;
import frc.robot.commands.ShooterTransportation.PIDSquishMotor;
import frc.robot.commands.Transportation.TransportationContorl;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransportation;
import frc.robot.subsystems.Transportation;

public class Shooting extends CommandBase {
  /**
   * Creates a new SchoolShooting.
   */

  private Automation auto;
  private CommandBase squishSpeed, transportation, flyWheel;

  public Shooting(Automation automation) {
    squishSpeed = new PIDSquishMotor(ShooterTransportation.getinstance());
    transportation = new TransportationContorl(Transportation.getinstance());
    flyWheel = new PIDFlyWheel(Shooter.getinstance());

    auto = automation;
    addRequirements(auto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flyWheel.schedule();
    squishSpeed.schedule();
    transportation.schedule();
    Shooter.getinstance().shootCounter = 0;
    flyWheel.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   {
      flyWheel.execute();
     if(Shooter.getinstance().isFlyWheelOnTraget()){
      squishSpeed.execute();
      transportation.execute();
      //Intake.getinstance().intakeMotorControl(-0.3);
     }else{
      //squishSpeed.cancel();
      transportation.schedule();
      squishSpeed.schedule();
      transportation.cancel();
      squishSpeed.cancel();
      //Intake.getinstance().intakeMotorControl(0);
     }
      
    
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transportation.schedule();
      squishSpeed.schedule();
      squishSpeed.cancel();
      transportation.cancel();
      flyWheel.cancel();
     // Intake.getinstance().intakeMotorControl(0);
    // System.out.println("hi");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
