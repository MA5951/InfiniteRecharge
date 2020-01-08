/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Chassis.MAPath;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Chassis extends SubsystemBase {
  private static final double KP_MApath_distance = 3.2e-3 ;
  private static final double KI_MApath_distance = 0;
  private static final double KD_MApath_distance = 1.1e-3 ;

  private static final double KP_MApath_angle = 5e-3;
  private static final double KI_MApath_angle = 0;
  private static final double KD_MApath_angle = 1.0e-4 ;

  private static final double KP_Vision_distance =2.8e-3;
  private static final double KI_Vision_distance = 1e-10;
  private static final double KD_Vision_distance = 1.9e-4;


  private static final double KP_Vision_angle = 3e-3; 
  private static final double KI_Vision_angle = 0.5e-6; 
  private static final double KD_Vision_angle = 1e-5;

  private static final double KP_VELOCITY_LEFT = 0.00006;
  private static final double KI_VELOCITY_LEFT = 0;
  private static final double KD_VELOCITY_LEFT = 0.000001;

  private static final double KP_VELOCITY_RIGHT = 0.000058;
  private static final double KI_VELOCITY_RIGHT = 0;
  private static final double KD_VELOCITY_RIGHT = 0.000001;

  private static final double anglePIDVisionSetInputRange = 44.5;
  private static final double anglePidMApathSetInputRange = 180;

  private  double distanceFromMeterPIDVison = 34.5142; // the precent of pixel from the limelight 1m from the target
  public static  double ticksPerMeter = 5350; // the number of ticks per meter
  public static  double RPM = 5676 * 4;
  private double angle;
  private double sign;
  private double modle = sign;
  private double TrapezeDistanceFromCircle = 72.7964;
  private static Chassis chassis;
  private  CANSparkMax leftFrontMotor;
  private  CANSparkMax leftMotor;
  
  private  CANSparkMax rightFrontMotor;
  private  CANSparkMax rightMotor;

  private  CANEncoder canEncoderright;
  private  CANEncoder canEncoderleft;
 
  private  Encoder leftEncoder;
  private  Encoder rightEncoder;
 
  private  AHRS navx;
  private  PIDController distancePidMApath; // PID controler of the distance in the pathfinder
  private  PIDController anglePidMApath; // PID controler of the angel in the pathfinder

  private  PIDController anglePIDVision; // the angel PID in the vison PID
  private  PIDController distancePIDVision; // the distce PID in the vison PID

  private  PIDController leftVelocityControl;
  private  PIDController rightVelocityControl;

  public static double[][] mainPath; // the array we us as the main Path in the MApath
 
  public static double[][] leftRocketPath1 = { new double[] { 2.175, 0, 0.3, 5, 0.3, 1 },
      new double[] { 3.35, 45, 0.3, 5, 0.2, 0.6 }, new double[] { 4.525, 90, 0.3, 5, 0.2, 0.6 },
      new double[] { 5.7, 135, 0.3, 5, 0.2, 0.6 }, new double[] { 6.875, 180, 0.3, 5, 0.2, 0.6 },
      new double[] { 9.25, 180, 0.3, 5, 0.20, 1 },

      new double[] { 6.875, 180, 0.3, 5, 0.10, 0.55 }, new double[] { 5.7, 135, 0.3, 5, 0.10, 0.55 },
      new double[] { 4.525, 90, 0.3, 5, 0.10, 0.55 }, new double[] { 3.35, 45, 0.3, 5, 0.10, 0.55 },
      new double[] { 2.175, 0, 0.3, 5, 0.15, 0.50 }, new double[] { 0, 0, 0.05, 5, 0.25, 0.55 }

  };
  public static double[][] leftRocketPath2 = new double[][] {
      // 1) distance
      // 2) angle
      // 3) distance Tolerance
      // 4) angle Tolerance
      // 5) Distance speed limit
      // 6) Angle Speed ​​Limit

      new double[] { 2.73, 0, 0.3, 10, 0.3, 0.9 }, new double[] { 4.73, 0, 0.3, 10, 0.3, 0.9 },
      new double[] { 7.73, 0, 0.3, 10, 0.3, 0.9 }, new double[] { 7.73, 92.14, 0.3, 10, 0, 0.7 },

      new double[] { 7.94, 98.09, 0.3, 10, 0.4, 0.7 }, new double[] { 8.040, 104.4, 0.3, 10, 0.4, 0.7 },
      new double[] { 8.14, 110.94, 0.3, 10, 0.4, 0.7 }, new double[] { 8.24, 117.54, 0.3, 10, 0.4, 0.7 },
      new double[] { 8.34, 124.04, 0.3, 10, 0.4, 0.7 }, new double[] { 8.44, 130.28, 0.3, 10, 0.4, 0.7 },
      new double[] { 8.549, 136.14, 0.3, 10, 0.4, 0.7 }, new double[] { 8.659, 141.54, 0.3, 10, 0.4, 0.7 },
      new double[] { 8.779, 146.44, 0.3, 10, 0.4, 0.7 }, new double[] { 8.909, 150.86, 0.3, 10, 0.4, 0.7 },
      new double[] { 9.04, 154.81, 0.3, 10, 0.4, 0.7 }, new double[] { 9.18, 158.34, 0.3, 10, 0.4, 0.7 },
      new double[] { 9.33, 161.48, 0.3, 10, 0.4, 0.7 }, new double[] { 9.49, 164.28, 0.3, 10, 0.4, 0.7 },
      new double[] { 9.66, 166.78, 0.3, 10, 0.4, 0.7 }, new double[] { 9.84, 169.02, 0.3, 10, 0.4, 0.7 },
      new double[] { 10.03, 171.03, 0.3, 10, 0.4, 0.7 }, new double[] { 10.22, 172.85, 0.3, 10, 0.4, 0.7 },
      new double[] { 10.44, 174.48, 0.3, 10, 0.4, 0.7 }, new double[] { 10.66, 175.97, 0.3, 10, 0.4, 0.7 },
      new double[] { 10.89, 177.32, 0.3, 10, 0.4, 0.7 }, new double[] { 11.13, 178.55, 0.3, 10, 0.4, 0.7 },
      new double[] { 11.38, 179.68, 0.3, 10, 0.4, 0.7 }, new double[] { 11.38, 89.12, 0.3, 10, 0, 0.7 },

      new double[] { 12.12, 0.64, 0.3, 10, 0.4, 0.7 }, new double[] { 12.35, 1.78, 0.3, 10, 0.4, 0.7 },
      new double[] { 12.57, 3.03, 0.3, 10, 0.4, 0.7 }, new double[] { 12.78, 4.39, 0.3, 10, 0.4, 0.7 },
      new double[] { 12.98, 5.89, 0.3, 10, 0.4, 0.7 }, new double[] { 13.17, 7.54, 0.3, 10, 0.4, 0.7 },
      new double[] { 13.35, 9.36, 0.3, 10, 0.4, 0.7 }, new double[] { 13.53, 11.38, 0.3, 10, 0.4, 0.7 },
      new double[] { 13.70, 13.62, 0.3, 10, 0.4, 0.7 }, new double[] { 13.86, 16.12, 0.3, 10, 0.4, 0.7 },
      new double[] { 14.01, 18.91, 0.3, 10, 0.4, 0.7 }, new double[] { 14.15, 22.02, 0.3, 10, 0.4, 0.7 },
      new double[] { 14.28, 25.51, 0.3, 10, 0.4, 0.7 }, new double[] { 14.41, 29.4, 0.3, 10, 0.4, 0.7 },
      new double[] { 14.53, 33.74, 0.3, 10, 0.4, 0.7 }, new double[] { 14.64, 38.53, 0.3, 10, 0.4, 0.7 },
      new double[] { 14.75, 43.79, 0.3, 10, 0.4, 0.7 }, new double[] { 14.86, 49.47, 0.3, 10, 0.4, 0.7 },
      new double[] { 14.96, 55.52, 0.3, 10, 0.4, 0.7 }, new double[] { 15.06, 61.81, 0.3, 10, 0.4, 0.7 },
      new double[] { 15.16, 68.22, 0.3, 10, 0.4, 0.7 }, new double[] { 15.26, 74.57, 0.3, 10, 0.4, 0.7 },
      new double[] { 15.36, 80.73, 0.3, 10, 0.4, 0.7 }, new double[] { 15.46, 86.56, 0.3, 10, 0.4, 0.7 },
      new double[] { 23.2, 179.92, 0.05, 5, 0, 0.7 } };
  public static double[][] leftRocketPath3 = new double[][] {
      // 1) distance
      // 2) angle
      // 3) distance Tolerance
      // 4) angle Tolerance
      // 5) Distance speed limit
      // 6) Angle Speed ​​Limit
      new double[] { 0, 0, 0.1, 5, 0.5, 0.7 }, new double[] { 2, 0, 0.1, 5, 0.4, 0.7 },
      new double[] { 6.7, 45, 3.175, 10, 0.3, 0.7 }, new double[] { 6.7, 90, 4.35, 10, 0.3, 0.7 },
      new double[] { 6.7, 135, 5.525, 10, 0.3, 0.7 }, new double[] { 6.7, 180, 0.05, 10, 0.3, 0.7 },
      new double[] { 8.7, 180, 0.1, 5, 0.4, 0.7 } };

  private Chassis() {

    leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_MOTOR,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMotor = new CANSparkMax(Constants.LEFT_MOTOR, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_MOTOR,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.RIGHT_MOTOR, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    canEncoderleft = leftFrontMotor.getEncoder();
    canEncoderright = rightFrontMotor.getEncoder();

    leftMotor.follow(leftFrontMotor);
    rightMotor.follow(rightFrontMotor);

    leftFrontMotor.setInverted(true);
    leftMotor.setInverted(true);

    navx = new AHRS(Port.kMXP);

    leftEncoder = new Encoder(Constants.LEFT_CHASSIS_ENCODER_A, Constants.LEFT_CHASSIS_ENCODER_B, false,
        EncodingType.k4X);
    rightEncoder = new Encoder(Constants.RIGHT_CHASSIS_ENCODER_A, Constants.RIGHT_CHASSIS_ENCODER_B, false,
        EncodingType.k4X);

    leftEncoder.setDistancePerPulse(1);
    rightEncoder.setDistancePerPulse(1);
    canEncoderleft.setVelocityConversionFactor(1);
    canEncoderright.setVelocityConversionFactor(1);

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
    return canEncoderright.getVelocity();
  }

  // updat the value in the smart dash bord
  public void value() {

    SmartDashboard.putNumber("angle", fixedAngle());
    SmartDashboard.putNumber("pathnum", MAPath.pathnum);
    SmartDashboard.putNumber("stage", MAPath.stage);
    SmartDashboard.putNumber("PIDVisionAngel", PIDVisionAngel());
    SmartDashboard.putNumber("distacne",
        ((leftEncoder.getDistance() + rightEncoder.getDistance()) / 2) / ticksPerMeter);
    SmartDashboard.putNumber("angelSetPoint", anglePidMApath.getSetpoint());
    SmartDashboard.putNumber("angleEror", angleEror());
    SmartDashboard.putNumber("DistanceSetPoint", distancePidMApath.getSetpoint() / ticksPerMeter);

    SmartDashboard.putNumber("angelSetPointvison", anglePIDVision.getSetpoint());
    SmartDashboard.putNumber("DistanceSetPointvison", distancePIDVision.getSetpoint());
    SmartDashboard.putNumber("Distancevison", distance());

    SmartDashboard.putNumber("leftEncoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("rightEncoder", rightEncoder.getDistance());

    SmartDashboard.putNumber("leftVelocityControlSetPoint", leftVelocityControl.getSetpoint());
    SmartDashboard.putNumber("rightVelocityControlSetPoint", rightVelocityControl.getSetpoint());

    

  }

  public void rampRate( double rampRate) {
    rightFrontMotor.setOpenLoopRampRate(rampRate);
    rightMotor.setOpenLoopRampRate(rampRate);
    leftFrontMotor.setOpenLoopRampRate(rampRate);
    leftMotor.setOpenLoopRampRate(rampRate);
  }

  public double leftVelocityControlSetPoint( double leftSetpoint) {
    return leftVelocityControl.calculate(lefttVelocityControlRPM(), leftSetpoint);
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
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
  }

  // return the main path
  public static double[][] getPath() {
    return mainPath;
  }

  public double angle() {
    return navx.getAngle();
  }

  public double fixedAngle() {
    try {
      angle = angle();
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
    leftEncoder.reset();
    rightEncoder.reset();
  }

  // pid vison distance
  public double distance() {
   //double distance = Math.sqrt(distanceFromMeterPIDVison / Robot.y) * 100;
  return (1833.48 / (0.00222335 * Math.pow(Robot.y, 2) + 0.228073 * Robot.y + 2.50245)) + 55.9811;
   
  }

  public double PIDVisionAngel(){
    return Math.atan(
        TrapezeDistanceFromCircle * Math.cos( Robot.x )
        / (distance() * (TrapezeDistanceFromCircle * Math.sin(Robot.x) -1 ))
      ); //credit to Ron Sanderovich
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

  // the PIDvison
  public void PIDvision( double angleSetpoint,  double distanceSetpoint) {
     double angel = anglePIDVisionOutput(angleSetpoint);
     double distacne = distancePIDVisionOutput(distanceSetpoint);
    if (Robot.area == 0.0) {
      reset();
      return;
    }
     double w = (100 - Math.abs(angel * 100)) * (distacne) + distacne * 100;
     double v = (100 - Math.abs(distacne * 100)) * (angel) + angel * 100;
    rightFrontMotor.set(-(v - w) / 100);
    leftFrontMotor.set((v + w) / 100);
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

  // MApath
  public void pathfinder() {
     double angel = angleMApathPIDOutput();
     double distance = distanceMApathPIDOutput();
    try {
       double w = (100 - Math.abs((angel * mainPath[MAPath.stage][5])) * 100)
          * (distance * mainPath[MAPath.stage][4]) + (distance * mainPath[MAPath.stage][4]) * 100;

       double v = (100 - Math.abs((distance * mainPath[MAPath.stage][4]) * 100))
          * (angel * mainPath[MAPath.stage][5]) + (angel * mainPath[MAPath.stage][5]) * 100;

      tankDrive((-(v + w) / 100), ((v - w) / 100));
    } catch ( Exception e) {

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
