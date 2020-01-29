/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.SparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.I2C.Port;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class Balance extends SubsystemBase {
  /**
   * Creates a new Balance.
   */
   private CANSparkMax balanceMotor;

   private CANDigitalInput IRLeft;
   private CANDigitalInput IRRight;

   private AHRS navx;

   private static final double KP_BALANCE = 0;
   private static final double  KI_BALANCE = 0;
   private static final double  KD_BALANCE = 0;
   private static final double angleTolorance = 0.5;

   private PIDController balancePidController;

   private static Balance balance;


   
   
   
  public Balance() {

    balanceMotor = new CANSparkMax(BalanceConstants.SPARK_MAX_BALANCE_PORT,MotorType.kBrushless);

    IRLeft = balanceMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    IRRight = balanceMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    navx = new AHRS(Port.kOnboard );

    balancePidController = new PIDController(KP_BALANCE,KI_BALANCE,KD_BALANCE);
    balancePidController.setTolerance(angleTolorance);

  }

  public double balancePidControllerSetPoint(double balanceSetPoint)
  {
    return MathUtil.clamp(balancePidController.calculate(navx.getPitch(), balanceSetPoint), -1, 1); // TODO
  }

  public boolean isBalancePIDOnTarget(){
    return balancePidController.atSetpoint();
  }

  //sets speed by driver conrtoll (joystick)
  public void controlBalanceMotor(double speed){
    balanceMotor.set(speed);
  }

  public void resetNavx() {
    navx.reset();
  }

  public void changeModeSparkMaxBrake(){
    balanceMotor.setIdleMode(IdleMode.kBrake);
  }

  public void changeModeSparkMaxCoast(){
    balanceMotor.setIdleMode(IdleMode.kCoast);
  }

  public boolean isCloseTotargetfromRight()
  {
    return IRRight.get();
  }

  public boolean isCloseTotargetfromLeft()
  {
    return IRLeft.get();
  }

  public static Balance getinstance()
  {
    if (balance == null) {
      balance = new Balance();
    }
    return balance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
