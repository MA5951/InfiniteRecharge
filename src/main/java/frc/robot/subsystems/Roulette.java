/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RoulleteConstants;

public class Roulette extends SubsystemBase {
  /**
   * The Roullete Subsystem
   */

  
  private static Roulette roulette;

  private TalonSRX rouletteMotor;
  private ColorSensorV3 colorSensor;
  private Color detectedColor = colorSensor.getColor();
  private Color lastColor = detectedColor;

  private String setpointColorString;

  public static int roundThreeColorSetpoint;

  private PIDController rouletteSpinControl;

  public static final int KP_ROULETTE = 0;
  public static final int KI_ROULETTE = 0;
  public static final int KD_ROULETTE = 0;
  public static final int TOLERANCE = 0;
  private int ticks = 0;

  private Roulette() {
    rouletteMotor = new TalonSRX(RoulleteConstants.ROULETTE_MOTOR);
    rouletteMotor.setNeutralMode(NeutralMode.Brake);
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    rouletteSpinControl = new PIDController(KP_ROULETTE, KI_ROULETTE, KD_ROULETTE);
    rouletteSpinControl.setTolerance(TOLERANCE);
    rouletteSpinControl.enableContinuousInput(0, 3);
  }

  private void displayValues() {
    /**
     * Display values into the shuffleboard
     */
    SmartDashboard.putNumber("blue", detectedColor.blue);
    SmartDashboard.putNumber("green", detectedColor.green);
    SmartDashboard.putNumber("red", detectedColor.red);
    SmartDashboard.putString("SetpointColor", setpointColorString);
    SmartDashboard.putNumber("Ticks", ticks);
  }

  public void countTicks() {
    /**
     * Increase ticks by one every time color is changed
     */
    if (detectedColor != lastColor) {
      ticks++;
      lastColor = detectedColor;
    }
  }

  public void resetTicks() {
    /**
     * Reset the ticks
     */
    ticks = 0;
  }
  
  public double spinPidOutput(double setpointColor) {
    return MathUtil.clamp(rouletteSpinControl.calculate(ticks, setpointColor), -1 , 1);
  }

  public void controlSpeed(double speed) {
    rouletteMotor.set(ControlMode.PercentOutput, speed);

  }

  public boolean isPIDOnTarget() {
    return rouletteSpinControl.atSetpoint();
  }

  public int blue() {
    return 0;
  }

  public int green() {
    return 1;
  }

  public int red() {
    return 2;
  }

  public int yellow() {
    return 3;
  }


  public static Roulette getinstance() {
    if (roulette == null) {
      roulette = new Roulette();
    }
    return roulette;
  }

  @Override
  public void periodic() {
    displayValues();
  }
}
