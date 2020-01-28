/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transportation extends SubsystemBase {
  /**
   * Creates a new Transportation.
   */
  private static Transportation transportation;

  private WPI_VictorSPX transportationMotor;
  private Transportation() {
      transportationMotor = new WPI_VictorSPX(Constants.transportationMotor);
  }

  public void transportationControl(double power) {
    transportationMotor.set(power);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public static Transportation getinstance() {
    if (transportation == null) {
      transportation = new Transportation();
    }
    return transportation;
  }
}
