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
import frc.robot.commands.Chassis.IntakeOpenClose;
import frc.robot.commands.Chassis.IntakePullPush;
import frc.robot.subsystems.Intake;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static Intake intake = Intake.getinstance();

  public static XboxController OperatingJoystick = new XboxController(2);
  public static Joystick leftJoystick = new Joystick(0);
  public static Joystick rightJoystick = new Joystick(1);

  private JoystickButton pushIntake = new JoystickButton(OperatingJoystick, Constants.pushIntakeButton);
  private JoystickButton pullIntake = new JoystickButton(OperatingJoystick, Constants.pullIntakeButton);
  private JoystickButton openCloseIntake = new JoystickButton(OperatingJoystick, Constants.openCloseIntakeButton);



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
      pushIntake.whileHeld(new IntakePullPush(-0.5, intake));
      pullIntake.whileHeld(new IntakePullPush(0.5, intake));
      openCloseIntake.whenPressed(new IntakeOpenClose(intake));
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
