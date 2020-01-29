/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AngleChangeSolenoidShooter extends InstantCommand {
private Shooter shooter;

  public AngleChangeSolenoidShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 if(shooter.isPistonOpen()){
   shooter.shooterAngle = 0; //TODO
 }else{
  shooter.shooterAngle = 0; //TODO
 }
    this.shooter.OpenAngleChangeSolenoid(!shooter.isPistonOpen());
  }
}
