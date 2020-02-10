/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ShooterTransportation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTransportation;
import frc.robot.subsystems.Transportation;

public class PIDSquishMotor extends CommandBase {
  /**
   * Creates a new PIDSquishMotor.
   */
  private ShooterTransportation shooterTransportation;
  private Shooter shooter;

  public PIDSquishMotor( ShooterTransportation shooterTransportation) {
    this.shooterTransportation = shooterTransportation;
    this.shooter = Shooter.getinstance();
   


     addRequirements(this.shooterTransportation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = shooterTransportation.squishMotorSpeedOutput();
    //if(shooter.getRate()>shooter.getPIDSetpoint() -100 && shooter.getRate()< shooter.getPIDSetpoint()+100){
      
    if(shooterTransportation.getMotorCurrnet() < -30) {
      shooterTransportation.controlSquishMotor(0.95);
      Timer.delay(0.2);
    } else {
      this.shooterTransportation.controlSquishMotor(-0.95);
    
  }
 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      this.shooterTransportation.controlSquishMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
