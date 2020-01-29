/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Roulette;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static double x;
  public static double v;
  public static double area;
  public static double y;
  public static double tlong;
  public static double yaw1;
  public static double distanceFromTargetLimelightX;
  public static double distanceFromTargetLimelightY;
  public static double finalLimelightAng;
  public static int path;

  private RobotContainer m_robotContainer;
 

  private Roulette roulette = Roulette.getinstance();
  private String gameData = DriverStation.getInstance().getGameSpecificMessage();
  public static int setpointColor;
  public static String colorString = "Unknown";


  public void getColorFromFMS() {
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          // Blue case code
          setpointColor = roulette.color("blue");
          colorString = "blue";
          break;
        case 'G' :
          // Green case code
          setpointColor = roulette.color("green");
          colorString = "green";
          break;
        case 'R' :
          //red case code
          setpointColor = roulette.color("red");
          colorString = "red";
          break;
        case 'Y' :
          //Yellow case code
          setpointColor = roulette.color("yellow");
          colorString = "yellow";
          break;
        default :
          //This is corrupt data
          break;
      }
    }
  }


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    final NetworkTableEntry tx = table.getEntry("tx");
    final NetworkTableEntry tv = table.getEntry("tv");
    final NetworkTableEntry ta = table.getEntry("ta");
    final NetworkTableEntry ty = table.getEntry("ty");
    final NetworkTableEntry tlong1 = table.getEntry("tlong");
    final NetworkTableEntry yaw = table.getEntry("camtran");

    // read values periodically
   
 
    getColorFromFMS();

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    Chassis.getinstance().setidilmodeBrake();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
     CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
