/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.Shooter.PIDFlyWheel;
import frc.robot.commands.ShooterTransportation.PIDSquishMotor;
import frc.robot.commands.Transportation.TransportationContorl;
import frc.robot.subsystems.Automation;
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
      squishSpeed.execute();
     if(Shooter.getinstance().isFlyWheelOnTraget()){
      transportation.execute();
     }else{
      //squishSpeed.cancel();
      transportation.cancel();
     }
      
    
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
      squishSpeed.cancel();
      transportation.schedule();
      transportation.cancel();
      flyWheel.cancel();
    // System.out.println("hi");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
