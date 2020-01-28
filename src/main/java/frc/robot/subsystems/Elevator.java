/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Elevator extends SubsystemBase {
  
  private static final double KP_ELEVATOR = 0.01;
  private static final double KI_ELEVATOR = 0;
  private static final double KD_ELEVATOR = 0;

  public static double ticksPerMeter = 22000; // the number of ticks per meter
  public static double RPM = 5240;
  public static int countsPerRev = 1;
  public static int distancePerPulse =1;
  public static double elevatorHightPIDSetInputRange = 1;

  private  CANSparkMax elevatorMotor;
  private Encoder encoderElevator;
  private  PIDController elevatorHightPID; // PID controler for hight
  private DoubleSolenoid elevatorPiston;

  private CANDigitalInput limitSwitchLeft;
  private CANDigitalInput limitSwitchRight;

  private static Elevator elevator;

  private Elevator() {
    //Motor
    elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    //elevatorMotor.setInverted(true);

    //Encoder
    encoderElevator = new Encoder(Constants.ELEVATOR_ENCODER_A, Constants.ELEVATOR_ENCODER_B);

    //encoderElevator.setDistancePerPulse(distancePerPulse);

    //PID
    elevatorHightPID = new PIDController(KP_ELEVATOR, KI_ELEVATOR, KD_ELEVATOR);

    elevatorHightPID.enableContinuousInput(-elevatorHightPIDSetInputRange, elevatorHightPIDSetInputRange);
    //elevatorHightPID.setTolerance(1);

    //Piston
    elevatorPiston = new DoubleSolenoid(Constants.ELEVATOR_DOUBLE_SOLENOID_A, Constants.ELEVATOR_DOUBLE_SOLENOID_B);

    //Limit Switches

    limitSwitchLeft = new CANDigitalInput(elevatorMotor,LimitSwitch.kForward,LimitSwitchPolarity.kNormallyOpen );
    limitSwitchRight = new CANDigitalInput(elevatorMotor,LimitSwitch.kForward,LimitSwitchPolarity.kNormallyOpen);
    }

    
  public static Elevator getinstance() {
      if (elevator == null) {
        elevator = new Elevator();
      }
      return elevator;
      }
  
  // updat the value in the smart dash bord
    public void value() {
      SmartDashboard.putNumber("Elevator Error", elevatorHightPIDError());
    }
    @Override
    public void periodic() {
      value();
  
    }
  // pid reset
  public void elevatorEncoderReset() {
    encoderElevator.reset();
    
  }

  public double getElevatorHightPIDOutput( double setpoint) {
    return elevatorHightPID.calculate(Robot.x, setpoint);
  }

  public void setElevatorHightSetpoint(double setpoint){
    elevatorHightPID.setSetpoint(setpoint);
  }

  public void setElvatorMotorSpeed(double speed){
    elevatorMotor.set(speed);
  }

  public double elevatorHightPIDError() {
    
    return elevatorHightPID.getPositionError();
    
  }

  public void openElevatorPiston(){
    elevatorPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void closeElevatorPiston(){
    elevatorPiston.set(DoubleSolenoid.Value.kReverse);
  } 


  public boolean isLeftLimitSwitchPressed(){
    return limitSwitchLeft.get();
  }
  public boolean isRightLimitSwitchPressed(){
    return limitSwitchRight.get();
  }


}
