/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static Shooter shooter = Shooter.getinstance(); 
  public static XboxController OperatingJoystick = new XboxController(2);
  public static Joystick leftJoystick = new Joystick(0);
  public static Joystick rightJoystick = new Joystick(1);

private static JoystickButton AngleChangeSolenoidShooter = new JoystickButton(OperatingJoystick, 1);
private static JoystickButton PIDFlyWheel = new JoystickButton(OperatingJoystick, 2);
private static JoystickButton PIDSquishMotor = new JoystickButton(OperatingJoystick, 3);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  AngleChangeSolenoidShooter.whenPressed(new frc.robot.commands.Shooter.AngleChangeSolenoidShooter(shooter));
  PIDFlyWheel.whileHeld(new frc.robot.commands.Shooter.PIDFlyWheel(shooter));
  PIDSquishMotor.whileHeld(new frc.robot.commands.Shooter.PIDSquishMotor(shooter));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   //public Command getAutonomousCommand() {
  
    // return  
   //}
}
