/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Transportation extends SubsystemBase {
  /**
   * Creates a new Transportation.
   */
  private static Transportation transportation;

  private TalonSRX transportationMotor;
  private Transportation() {
      transportationMotor = new TalonSRX(TransportationConstants.transportationMotor);
  }

  public void transportationControl(double power) {
    transportationMotor.set(ControlMode.PercentOutput, power);
  }

  public double getMotorCurrnet() {
    return transportationMotor.getStatorCurrent();
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
