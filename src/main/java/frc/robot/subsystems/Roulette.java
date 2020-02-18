/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.Constants.RouletteConstants;

public class Roulette extends SubsystemBase {
  /**
   * The Roullete Subsystem
   */

  private static Roulette roulette;

  private VictorSPX rouletteMotor;

  private ColorSensorV3 colorSensor;

  private Color detectedColor;
  private Color lastColor;
  private ColorMatch colorMatcher;
  private ColorMatchResult match;

  private final Color blue;
  private final Color green;
  private final Color red;
  private final Color yellow;
  private Solenoid roulettSolenoid;
  public static int roundThreeColorSetpoint;

  private PIDController rouletteSpinControl;

  public static final double KP_ROULETTE = 0.05;
  public static final double KI_ROULETTE = 0;
  public static final double KD_ROULETTE = 0;
  public static final int TOLERANCE = 0;
  private int ticks = 0;

  private Roulette() {
    rouletteMotor = new VictorSPX(RouletteConstants.ROULETTE_MOTOR);
    rouletteMotor.setNeutralMode(NeutralMode.Brake);
    colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    roulettSolenoid = new Solenoid(RouletteConstants.ROULETTE_SOLENOID);

    rouletteSpinControl = new PIDController(KP_ROULETTE, KI_ROULETTE, KD_ROULETTE);
    rouletteSpinControl.setTolerance(TOLERANCE);

    detectedColor = colorSensor.getColor();
    lastColor = detectedColor;
    colorMatcher = new ColorMatch();

    blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
    green = ColorMatch.makeColor(0.197, 0.561, 0.240);
    red = ColorMatch.makeColor(0.561, 0.232, 0.114);
    yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);

    colorMatcher.addColorMatch(blue);
    colorMatcher.addColorMatch(green);
    colorMatcher.addColorMatch(red);
    colorMatcher.addColorMatch(yellow);
  }

  private void displayValues() {
    /**
     * Display values into the shuffleboard
     */
    SmartDashboard.putString("Color", Robot.colorString);
    SmartDashboard.putNumber("Ticks", ticks);
  }

  public int getCurrentColor() {
    match = colorMatcher.matchClosestColor(colorSensor.getColor());
    if (match.color == blue) {
      return 0;
    } else if (match.color == red) {
      return 1;
    } else if (match.color == green) {
      return 2;
    } else if (match.color == yellow) {
      return 3;
    } 
    return 0;
    
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

  public void ticksControl(boolean enable) {
    if (enable) {
      rouletteSpinControl.enableContinuousInput(0, 3);
    } else {
      rouletteSpinControl.disableContinuousInput();
    }
  }

  public void resetTicks() {
    /**
     * Reset the ticks
     */
    ticks = 0;
  }

  public double spinPidOutput(double setpointColor) {
    return MathUtil.clamp(rouletteSpinControl.calculate(ticks, setpointColor), -1, 1);
  }

  public void controlSpeed(double speed) {
    rouletteMotor.set(ControlMode.PercentOutput, speed);

  }

  public int color(String color) {
    if (color == "blue") {
      return 0;
    } else if (color == "green") {
      return 1;
    } else if (color == "red") {
      return 2;
    } else if (color == "yellow") {
      return 3;
    }
    // If nothing is true (because you have to return something)
    return 0;

  }

  public boolean isPIDOnTarget() {
    return rouletteSpinControl.atSetpoint();
  }

  public static Roulette getinstance() {
    if (roulette == null) {
      roulette = new Roulette();
    }
    return roulette;
  }

  public void controlroulettSolenoid(boolean value) {
    roulettSolenoid.set(value);
  }

  @Override
  public void periodic() {
    displayValues();
    countTicks();
    match = colorMatcher.matchClosestColor(detectedColor);
    // TODO Check where to call ticks count function
  }
}