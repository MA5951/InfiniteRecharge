/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.BalanceConstants;
import com.revrobotics.CANDigitalInput;

public class Balance extends SubsystemBase {
  /**
   * Creates a new Balance.
   */
  private CANSparkMax balanceMotor;

  private static Balance balance;

  private Balance() {

    balanceMotor = new CANSparkMax(BalanceConstants.SPARK_MAX_BALANCE_PORT, MotorType.kBrushless);

  }

  // sets speed by driver conrtoll (joystick)
  public void controlBalanceMotor(double speed) {
    balanceMotor.set(speed);
  }

  public static Balance getinstance() {
    if (balance == null) {
      balance = new Balance();
    }
    return balance;
  }

  @Override
  public void periodic() {

  }
}