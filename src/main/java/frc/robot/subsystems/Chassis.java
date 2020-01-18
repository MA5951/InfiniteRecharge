/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.commands.Chassis.PIDVisionAngel;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Chassis extends SubsystemBase {
  private static final double PIDmultiplayr = 4.12;
  private static final double KP_MApath_distance = 2.3e-3 / PIDmultiplayr;
  private static final double KI_MApath_distance = 0;
  private static final double KD_MApath_distance  = 2.7e-4 / PIDmultiplayr;

  private static final double KP_MApath_angle = 2.7e-2 / PIDmultiplayr ;
  private static final double KI_MApath_angle = 0;
  private static final double KD_MApath_angle = 4e-3 / PIDmultiplayr;

  private static final double KP_Vision_distance =2.8e-3 / 1.6875;
  private static final double KI_Vision_distance = 1e-10 *1.6875;
  private static final double KD_Vision_distance = 1.9e-4 * 1.6875;


  private static final double KP_Vision_angle = 3e-3 * 1.6875; 
  private static final double KI_Vision_angle = 0.5e-6 * 1.6875; 
  private static final double KD_Vision_angle = 1e-5 * 1.6875;

  private static final double KP_VELOCITY_LEFT = 0.000058 *6;
  private static final double KI_VELOCITY_LEFT = 0;
  private static final double KD_VELOCITY_LEFT = 0.000001 *6;

  private static final double KP_VELOCITY_RIGHT = 0.000058 *6 ;
  private static final double KI_VELOCITY_RIGHT = 0;
  private static final double KD_VELOCITY_RIGHT = 0.000001 *6;

  private static final double anglePIDVisionSetInputRange = 44.5;
  private static final double anglePidMApathSetInputRange = 180;
  
  public static  double ticksPerMeter = 22000; // the number of ticks per meter
  public static  double RPM = 5240;
  private double angle;
  private double sign;
  private double modle = sign;

  private static Chassis chassis;
  private  CANSparkMax leftFrontMotor;
  private  CANSparkMax leftMotor;
  
  private  CANSparkMax rightFrontMotor;
  private  CANSparkMax rightMotor;

  private  CANSparkMax leftFrontMotor1;
  private  CANSparkMax leftMotor1;
  
  private  CANSparkMax rightFrontMotor1;
  private  CANSparkMax rightMotor1;

  private CANEncoder canEncoderRightCIMcoder;
  private CANEncoder canEncoderLeftCIMcoder;

private CANEncoder canEncoderleft;
private CANEncoder canEncoderRight;

 
  private  AHRS navx;
  private  PIDController distancePidMApath; // PID controler of the distance in the pathfinder
  private  PIDController anglePidMApath; // PID controler of the angel in the pathfinder

  private  PIDController anglePIDVision; // the angel PID in the vison PID
  private  PIDController distancePIDVision; // the distce PID in the vison PID

  private  PIDController leftVelocityControl;
  private  PIDController rightVelocityControl;

  public static double[][] mainPath; // the array we us as the main Path in the MApath
 
  public static double[][] leftRocketPath1 = { 
//new double[]{0, 180, 0.3, 2, 0, 0.7},
new double[]{0.87, 4.5, 0.3, 10, 0.35 , 1},
new double[]{1.390, -32.28, 0.3, 10, 0.35 , 1},
new double[]{1.900, -67.46, 0.3, 10, 0.35 , 1},
new double[]{2.400, -77.49, 0.3, 10, 0.35 , 1},
new double[]{2.820, -62.85, 0.3, 10, 0.35 , 1},
new double[]{3.270, -11, 0.3, 10, 0.35 , 1},
new double[]{3.6, 0, 0.3, 2, 0.3 , 1},
new double[]{7, 0, 0.05, 3, 0.3, 1}
  };

  public static double[][] try_path = {
new double[]{0.8 , -5, 0.3, 10, 0.2, 1},
new double[]{1.35, -32.54,0.3, 10, 0.25 , 0.65 },
new double[]{1.84, -68.24,0.3, 10,0.25 , 0.65 },
new double[]{2.33, -78.06,0.3, 10,0.25 , 0.65 },
new double[]{2.72, -62.32,0.3, 10,0.25 , 0.65 },
new double[]{3.16, 0,0.1, 2 ,0.2 , 0.4 },
new double[]{4.00, 0,0.3, 2 ,0.2 , 0.4 },
new double[]{6.9, 0, 0.1, 2, 0.2, 0.45},
new double[]{3.18, 0.0, 0.3, 5, 0.2, 0.4},
new double[]{2.21, -4.2, 0.3, 5, 0.25 , 0.65},
new double[]{1.58, -35.82, 0.3, 5, 0.25 , 0.65},
new double[]{1.04, -65.12, 0.3, 5, 0.25 , 0.65},
new double[]{0.570, -74.53, 0.3, 5, 0.25 , 0.65},
new double[]{0.1, -52.27, 0.3, 2, 0.25 , 0.65},
new double[]{-0.1, -25.27, 0.3, 2, 0.25 , 0.65},
new double[]{-0.5, 0, 0.3, 2, 0.2, 0.4},
new double[]{-3, 0, 0.05, 2, 0.2, 0.4}
    
  };
  public static double[][] leftRocketPath2 = new double[][] {
new double[]{0 , 45 , 0.05 , 2 , 0 , 1},
  };
  public static double[][] leftRocketPath3 = new double[][] {
      // 1) distance
      // 2) angle
      // 3) distance Tolerance
      // 4) angle Tolerance
      // 5) Distance speed limit
      // 6) Angle Speed ​​Limit
      new double[] { 0, 90, 0, 2, 0, 1 }, };

  private Chassis() {

    leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_MOTOR,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMotor = new CANSparkMax(Constants.LEFT_MOTOR, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_MOTOR,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.RIGHT_MOTOR, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    leftFrontMotor1 = new CANSparkMax(5,com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed);
    leftMotor1 = new CANSparkMax(6,com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed);

    rightFrontMotor1 = new CANSparkMax(9,com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed);
        rightMotor1 = new CANSparkMax(8, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed);

    canEncoderLeftCIMcoder = leftMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 1);
    canEncoderRightCIMcoder = rightMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 1);
    canEncoderRightCIMcoder.setInverted(true);

    canEncoderRight = rightFrontMotor.getEncoder();
    canEncoderleft = leftFrontMotor.getEncoder();

    canEncoderRightCIMcoder.setPositionConversionFactor(1);
    canEncoderLeftCIMcoder.setPositionConversionFactor(1);

    leftMotor.follow(leftFrontMotor);
    rightMotor.follow(rightFrontMotor);

    leftMotor.setInverted(true);
    leftFrontMotor.setInverted(true);

    navx = new AHRS(Port.kMXP);
  

  
    // the distance PID vison
    distancePIDVision = new PIDController(KP_Vision_distance, KI_Vision_distance, KD_Vision_distance);

    // the distance PID Pathfinder
    distancePidMApath = new PIDController(KP_MApath_distance, KI_MApath_distance, KD_MApath_distance);

    // the angel PID pathfinder
    anglePidMApath = new PIDController(KP_MApath_angle, KI_MApath_angle, KD_MApath_angle);

    // the angel PID vison
    anglePIDVision = new PIDController(KP_Vision_angle, KI_Vision_angle, KD_Vision_angle);

    leftVelocityControl = new PIDController(KP_VELOCITY_LEFT, KI_VELOCITY_LEFT, KD_VELOCITY_LEFT);

    rightVelocityControl = new PIDController(KP_VELOCITY_RIGHT, KI_VELOCITY_RIGHT, KD_VELOCITY_RIGHT);

    anglePIDVision.enableContinuousInput(-anglePIDVisionSetInputRange, anglePIDVisionSetInputRange);

    anglePidMApath.enableContinuousInput(-anglePidMApathSetInputRange, anglePidMApathSetInputRange);
    
   
    }

  public double lefttVelocityControlRPM() {
    return canEncoderleft.getVelocity();
  }

  public double rightVelocityControlRPM() {
    return canEncoderRight.getVelocity();
  }

  // updat the value in the smart dash bord
  public void value() {

    SmartDashboard.putNumber("angle",  navx.getFusedHeading()); //fixedAngle());
    SmartDashboard.putNumber("fixedAngle", fixedAngle());
    SmartDashboard.putNumber("pathnum", MAPath.pathnum);
    SmartDashboard.putNumber("stage", MAPath.stage);
    
    SmartDashboard.putNumber("distacne",
        average()/ ticksPerMeter);
    SmartDashboard.putNumber("angelSetPoint", anglePidMApath.getSetpoint());
    SmartDashboard.putNumber("angleEror", angleEror());
    SmartDashboard.putNumber("distanceEror", distanceEror());
    SmartDashboard.putNumber("DistanceSetPoint", distancePidMApath.getSetpoint() / ticksPerMeter);
 
    SmartDashboard.putNumber("angelSetPointvison", anglePIDVision.getSetpoint());
    SmartDashboard.putNumber("DistanceSetPointvison", distancePIDVision.getSetpoint());
    SmartDashboard.putNumber("Distancevison", distance());

    SmartDashboard.putNumber("getCompassHeading",  navx.getCompassHeading());
  
    SmartDashboard.putNumber("/limelight/pipeline", PIDVisionAngel.camMode);
    SmartDashboard.putNumber("leftVelocityControlSetPoint", leftVelocityControl.getSetpoint());
    SmartDashboard.putNumber("rightVelocityControlSetPoint", rightVelocityControl.getSetpoint());

    SmartDashboard.putNumber("canEncoderLeftCIMcoder", canEncoderLeftCIMcoder.getPosition());
    SmartDashboard.putNumber("canEncoderRightCIMcoder", canEncoderRightCIMcoder.getPosition());

  }

  public void rampRate( double rampRate) {
    rightFrontMotor.setOpenLoopRampRate(rampRate);
    rightMotor.setOpenLoopRampRate(rampRate);
    leftFrontMotor.setOpenLoopRampRate(rampRate);
    leftMotor.setOpenLoopRampRate(rampRate);
  }

  public double leftVelocityControlSetPoint( double leftSetpoint) {
    double val = leftVelocityControl.calculate(lefttVelocityControlRPM(), leftSetpoint);
    System.out.println(val + ", " + lefttVelocityControlRPM());
    return val;
  }

  public double rightVelocityControlSetPoint( double rightSetpoint) {
    return rightVelocityControl.calculate(rightVelocityControlRPM(), rightSetpoint);
  }

  public void resetVelocityControl() {
    leftVelocityControl.reset();
    rightVelocityControl.reset();

  }

  // the average of the encoders
  public double average() {
    return (canEncoderRightCIMcoder.getPosition() + canEncoderLeftCIMcoder.getPosition()) / 2;
  }

  // return the main path
  public static double[][] getPath() {
    return mainPath;
  }

 
  public double fixedAngle() {
    try {
      angle = navx.getAngle();
      sign = angle / Math.abs(angle);
      modle = sign * (Math.abs(angle) % 360);
      return -((180 - modle) % 360) + 180;
    } catch ( Exception e) {
      return 0;
    }
  }

  public void setidilmodeCoset() {
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setidilmodeBrake() {
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
  }

  // set the left and the right motors powers
  public void tankDrive( double leftSpeed,  double rightspped) {
    rightFrontMotor.set(rightspped);
    leftFrontMotor.set(leftSpeed);
  }

  // resat the value of the encoder and the navx
  public void resetValue() {
    navx.reset();
    canEncoderLeftCIMcoder.setPosition(0);
    canEncoderRightCIMcoder.setPosition(0);
  }

  // pid vison distance
  public double distance() {
  return (1833.48 / (0.00222335 * Math.pow(Robot.y, 2) + 0.228073 * Robot.y + 2.50245)) + 55.9811;
   
  }

  public boolean stopPid( double dis,  double angle) {
    return Math.abs(distancePIDVision.getPositionError()) < dis && Math.abs(anglePIDVision.getPositionError()) < angle;
  }
 
  // pid vosin
  public void reset() {
    distancePIDVision.reset();
    anglePIDVision.reset();
  }

  public double anglePIDVisionOutput( double setpoint) {
    return anglePIDVision.calculate(Robot.x, setpoint);
  }

  public double distancePIDVisionOutput( double setpoint) {
    return distancePIDVision.calculate(distance(), setpoint);
  }

  public void ArcadeDrive (double angel , double distacne  ){
    double w = (100 - Math.abs(angel * 150) ) * (distacne) + distacne * 50;
    double v = (100 - Math.abs(distacne * 50)) * (angel) + angel * 150;
 tankDrive(-(v+w)/ 100 , (v-w)/ 100);
  }

  // the PIDvison
  public void PIDvision( double angleSetpoint,  double distanceSetpoint  ) {
     double angel = anglePIDVisionOutput(angleSetpoint);
     double distacne = distancePIDVisionOutput(distanceSetpoint);
    if (Robot.area == 0.0) {
      reset();
      return;
    }
    ArcadeDrive(angel , distacne);
  }



  // setpoint of the PID vison
  public void setSetpoint( double angle,  double destination) {
    anglePIDVision.setSetpoint(angle);
    distancePIDVision.setSetpoint(destination);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // MAPath
  public void setpoint( double distancesetpoint,  double anglesetpoint,  double Speedlimitdistance,
       double Speedlimitangle) {
    anglePidMApath.setSetpoint(anglesetpoint);
    distancePidMApath.setSetpoint(distancesetpoint * ticksPerMeter);

    distancePidMApath.setP(KP_MApath_distance * Speedlimitdistance);
    distancePidMApath.setD(KD_MApath_distance * Speedlimitdistance);

    anglePidMApath.setP(KP_MApath_angle * Speedlimitangle);
    anglePidMApath.setD(KD_MApath_angle * Speedlimitangle);
  }

  public double angleEror() {
    return anglePidMApath.getPositionError();
  }

  public double distanceEror() {
    return distancePidMApath.getPositionError();
  }

  public double angleMApathPIDOutput() {
    return anglePidMApath.calculate(fixedAngle());
  }

  public double distanceMApathPIDOutput() {
    return distancePidMApath.calculate(average());
  }

public void proto(){
     leftFrontMotor1.set(1);
     leftMotor1.set(1);
  
     rightFrontMotor1.set(1);
     rightMotor1.set(1);
}
  // MApath
  public void pathfinder() {

    if(MAPath.stage <= mainPath.length - 1 ){
      double angel = angleMApathPIDOutput() * mainPath[MAPath.stage][5];
      double distance = distanceMApathPIDOutput() * mainPath[MAPath.stage][4];
      ArcadeDrive(angel, distance);
    }else{
      tankDrive(0, 0);
    }
  }

  public static Chassis getinstance() {
    if (chassis == null) {
      chassis = new Chassis();
    }
    return chassis;
  }

  @Override
  public void periodic() {
    value();

  }

}
