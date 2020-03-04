/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is used only for the automation commands
 */
public class Automation extends SubsystemBase {

  private static Automation auto;
  private Automation() {

  }

  public static Automation getinstance() {
    if (auto == null) {
      auto = new Automation();
    }
    return auto;
  }

  @Override
  public void periodic() {

  }
}
