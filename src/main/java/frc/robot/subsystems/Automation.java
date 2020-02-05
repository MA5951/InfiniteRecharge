/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.Automation.PreparationShooting;
import frc.robot.commands.Automation.Shooting;

import java.util.function.BooleanSupplier;

/**
 * This subsystem is used only for the automation commands
 */
public class Automation extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static Automation auto;
  public static BooleanSupplier isShootingPrepared;
  public static CommandBase shooting;
  public static CommandBase preparationShooting;

  private Automation(){
    shooting = new Shooting(auto.getinstance());
    preparationShooting = new PreparationShooting(auto.getinstance());
    
  }

  public static Automation getinstance() {
    if (auto == null) {
      auto = new Automation();
    }
    return auto;
  }

  @Override
  public void periodic() {
    isShootingPrepared = () -> Robot.isShootingPrepared;
  }
}
