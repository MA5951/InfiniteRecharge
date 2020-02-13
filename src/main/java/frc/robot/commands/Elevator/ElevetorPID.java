/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevetorPID extends CommandBase {
  /**
   * Creates a new ElevetorPID.
   */
  private Elevator elevator;
  private double setpoint;
  private double speed;
  private double lastTimeOnTarget;
  private double waitTime;
  public ElevetorPID(Elevator elevator , double setpoint , double waitTime) {
    this.elevator = elevator;
    this.setpoint = setpoint;
    this.waitTime = waitTime;
  addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = elevator.getElevatorPIDOutput(setpoint);
    elevator.setElvatorMotorSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setElvatorMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!elevator.isPIDElevetorOnTarget()) {
      lastTimeOnTarget = Timer.getFPGATimestamp();
    }
    return elevator.isPIDElevetorOnTarget() && Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime;
  }
  }