/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is used only for the automation commands
 */
public class Autonomous extends SubsystemBase {
  /**
   * Creates a new Autonomous.
   */
  public Autonomous() {

  }

  private static Autonomous auto;

  public static Autonomous getInstance() {
    if (auto == null) {
      auto = new Autonomous();
    }
    return auto;
  }

  @Override
  public void periodic() {
    SmartDashboard.setDefaultNumber("auto", 1);

  }
}
