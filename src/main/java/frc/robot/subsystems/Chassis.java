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

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ConstantsChassis;
import frc.robot.Robot;
import frc.robot.Path.Path;
import frc.robot.commands.Chassis.MAPath;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Chassis extends SubsystemBase {
  private static final double kLimelight3D = 25.625 * 2.54; // TODO

  private static final double KP_MApath_distance = 40e-6;
  private static final double KI_MApath_distance = 0;
  private static final double KD_MApath_distance = 20e-7;

  private static final double KP_MApath_angle = 1e-2;
  private static final double KI_MApath_angle = 0;
  private static final double KD_MApath_angle = 1e-3;

  private static final double KP_Vision_angle = 2.5e-2;
  private static final double KI_Vision_angle = 8e-4;
  private static final double KD_Vision_angle = 1e-3;

  private static final double anglePIDVisionSetInputRange = 44.5;
  private static final double anglePidMApathSetInputRange = 180;

  public static double ticksPerMeter = 22000; // the number of ticks per meter //TODO
  public static double RPM = 5700;
  private double angle;
  private double sign;
  private double modle = sign;

  private static Chassis chassis;

  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftMotor;

  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightMotor;

  private CANEncoder canEncoderRightCIMcoder;
  private CANEncoder canEncoderLeftCIMcoder;

  private CANEncoder canEncoderleft;
  private CANEncoder canEncoderRight;

  private AHRS navx;

  private PIDController distancePidMApath; // PID controler of the distance in the pathfinder
  private PIDController anglePidMApath; // PID controler of the angel in the pathfinder

  private PIDController anglePIDVision; // the angel PID in the vison PID

  private Chassis() {

    leftFrontMotor = new CANSparkMax(ConstantsChassis.LEFT_FRONT_MOTOR,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMotor = new CANSparkMax(ConstantsChassis.LEFT_MOTOR, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    rightFrontMotor = new CANSparkMax(ConstantsChassis.RIGHT_FRONT_MOTOR,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMotor = new CANSparkMax(ConstantsChassis.RIGHT_MOTOR,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    rightFrontMotor.setSmartCurrentLimit(60);
    rightMotor.setSmartCurrentLimit(60);
    leftFrontMotor.setSmartCurrentLimit(60);
    leftMotor.setSmartCurrentLimit(60);

    canEncoderLeftCIMcoder = leftMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 1);
    canEncoderRightCIMcoder = rightMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 1);
    canEncoderRightCIMcoder.setInverted(true);
    canEncoderLeftCIMcoder.setInverted(false);

    canEncoderRight = rightFrontMotor.getEncoder();
    canEncoderleft = leftFrontMotor.getEncoder();

    canEncoderRightCIMcoder.setPositionConversionFactor(1);
    canEncoderLeftCIMcoder.setPositionConversionFactor(1);

    canEncoderRight.setVelocityConversionFactor(1);
    canEncoderleft.setVelocityConversionFactor(1);

    leftMotor.follow(leftFrontMotor);
    rightMotor.follow(rightFrontMotor);

    leftMotor.setInverted(true);
    leftFrontMotor.setInverted(true);
    rightMotor.setInverted(false);
    rightFrontMotor.setInverted(false);

    navx = new AHRS(Port.kMXP);

    // the distance PID Pathfinder
    distancePidMApath = new PIDController(KP_MApath_distance, KI_MApath_distance, KD_MApath_distance);

    // the angel PID pathfinder
    anglePidMApath = new PIDController(KP_MApath_angle, KI_MApath_angle, KD_MApath_angle);

    // the angel PID vison
    anglePIDVision = new PIDController(KP_Vision_angle, KI_Vision_angle, KD_Vision_angle);
    anglePIDVision.setTolerance(2);

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
    SmartDashboard.putNumber("fixedAngle", fixedAngle());
    SmartDashboard.putNumber("stage", MAPath.stage);
    SmartDashboard.putNumber("distacne", average() / ticksPerMeter);
    SmartDashboard.putNumber("angelSetPoint", anglePidMApath.getSetpoint());
    SmartDashboard.putBoolean("PIDvisonOnTarget", anglePIDVision.atSetpoint());
    SmartDashboard.putNumber("DistanceSetPoint", distancePidMApath.getSetpoint() / ticksPerMeter);
    SmartDashboard.putNumber("Distancevison", distance());
  }

  public void rampRate(double rampRate) {
    rightFrontMotor.setOpenLoopRampRate(rampRate);
    rightMotor.setOpenLoopRampRate(0);
    leftFrontMotor.setOpenLoopRampRate(rampRate);
    leftMotor.setOpenLoopRampRate(0);
  }

  public double limelightAngleFinal() {
    if (Robot.distanceFromTargetLimelightX == 0) {
      return 0;
    } else {
      return 90 - Robot.yaw1
          - Math.toDegrees(Math.atan(((Math.abs(Robot.distanceFromTargetLimelightY * 2.54)) + kLimelight3D)
              / (Math.abs(Robot.distanceFromTargetLimelightX * 2.54))));
    }

  }

  // the average of the encoders
  public double average() {
    return (canEncoderRightCIMcoder.getPosition() + canEncoderLeftCIMcoder.getPosition()) / 2;
  }

  public double fixedAngle() {
    try {
      angle = navx.getYaw();
      sign = angle / Math.abs(angle);
      modle = sign * (Math.abs(angle) % 360);
      return -((180 - modle) % 360) + 180;
    } catch (Exception e) {
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
  public void tankDrive(double leftSpeed, double rightspped) {
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
    return (-1.3276 * Math.pow(10, 6) / (-2.43018 * Math.pow(Robot.y, 2) + -101.265 * Robot.y + -1854.19)); // TODO

  }

  // pid vosin
  public void reset() {
    anglePIDVision.reset();
  }

  public double anglePIDVisionOutput(double setpoint) {
    return MathUtil.clamp(anglePIDVision.calculate(Robot.x * -1, setpoint), -1, 1);
  }

  public void ArcadeDrive(double angel, double distacne) {
    double w = (100 - Math.abs(angel * 100)) * (distacne) + distacne * 100;
    double v = (100 - Math.abs(distacne * 100)) * (angel) + angel * 100;
    tankDrive((-(v + w) / 200), ((v - w) / 200));
  }

  // the PIDvison
  public void PIDvision(double angleSetpoint) {
    double power = anglePIDVisionOutput(angleSetpoint);
    tankDrive(-power, power);
  }

  public boolean isPIDVisionOnTarget() {
    return anglePIDVision.atSetpoint();
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // MAPath
  public void setpoint(double distancesetpoint, double anglesetpoint, double Speedlimitdistance,
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
    return MathUtil.clamp(anglePidMApath.calculate(fixedAngle()), -1.0, 1.0);
  }

  public double distanceMApathPIDOutput() {
    return MathUtil.clamp(distancePidMApath.calculate(average()), -1.0, 1.0);
  }

  // MApath
  public void pathfinder() {

    if (MAPath.stage <= Path.mainPath.length - 1) {
      double angel = angleMApathPIDOutput() * Path.mainPath[MAPath.stage][5];
      double distance = distanceMApathPIDOutput() * Path.mainPath[MAPath.stage][4];
      ArcadeDrive(angel, distance);
    } else {
      tankDrive(0, 0);
    }
  }

  public void leftcontrol(double power) {
    leftFrontMotor.set(power);
  }

  public void rightcontrol(double power) {
    rightFrontMotor.set(power);
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
