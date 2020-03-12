/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Limlight;

public class PIDVisionFeeder extends CommandBase {
  private Limlight limlight;
  private Chassis chassis;
 

  public PIDVisionFeeder(Limlight lm) {
    limlight = lm;

    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    chassis.rampRate(0);
    chassis.setidilmodeBrake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angel = chassis.anglePIDVisionOutput(0);
    double distacne = chassis.distancePIDVisionOutput(66);
    if(Robot.tshort > 2){
      chassis.ArcadeDrive(angel, distacne);
    }else{
      chassis.tankDrive(0, 0);
    }
  

    // chassis.PIDvision(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    if (interrupted) {
      chassis.tankDrive(0, 0);
      chassis.reset();
      chassis.setidilmodeBrake();
    } else {
      chassis.tankDrive(0, 0);
      chassis.reset();
      chassis.setidilmodeBrake();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * if (!chassis.isPIDVisionOnTarget()) { lastTimeOnTarget =
     * Timer.getFPGATimestamp(); } return chassis.isPIDVisionOnTarget() &&
     * Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime; }
     */
    return false;
  }
}