/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class PIDVisionAngel extends CommandBase {
  /**
   * Creates a new PIDVisionAngel.
   */
  public static int camMode = 0;
  double tpx = 320;
  public PIDVisionAngel() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camMode = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tL = Robot.tlong;
    //P1(PX1,PY1)
    camMode = 0;
    double pX1 = Robot.x / 29.8 * 160;
    double pY1 = Robot.y / 24.85 * 120;
    //P2(PX2,PY2)
    camMode = 2;
    new WaitCommand(0.5);
    double pX2 = Robot.x / 29.8 * 160;
    double pY2 = Robot.y / 24.85 * 120;
    //
    camMode = 0;
    //slope
    double mV = (pY1-pY2)/(pX1-pX2);
    double mH = -(1/mV);
    //x1,1 x1,2
    double x1 = pX2+(tL/(4*Math.sqrt(1+mH)));
    double x2 = pX2-(tL/(4*Math.sqrt(1+mH)));
    //
    double y1 = mH* (x1 - pX2) + pY2;
    double y2 = mH* (x2 - pX2) + pY2;
    //
    double tx1 = 2* x1/ tpx;
    double tx2 =  2*x2/ tpx;
System.out.println(mH);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
