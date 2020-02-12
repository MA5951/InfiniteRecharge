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
import frc.robot.Robot;
import frc.robot.commands.Automation.IntakeAutomation;
import frc.robot.commands.Automation.PreparationShooting;
import frc.robot.commands.Automation.Shooting;
import frc.robot.commands.Chassis.PIDVision;
import frc.robot.commands.Chassis.pathWriter;
import frc.robot.commands.Chassis.roulettePath;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Transportation.*;
import frc.robot.subsystems.Transportation;
import frc.robot.commands.Intake.IntakeOpenClose;
import frc.robot.commands.Intake.IntakePullPush;
import frc.robot.commands.Shooter.PIDFlyWheel;
import frc.robot.commands.ShooterTransportation.PIDSquishMotor;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Shooter;
/*
import frc.robot.commands.Roulette.roundTwoRoulettePID;
import frc.robot.commands.Roulette.roundThreeRoulettePID;
import frc.robot.subsystems.Roulette;
*/
import frc.robot.subsystems.ShooterTransportation;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
 
  
  private  Transportation transportation = Transportation.getinstance();
  private  Intake intake = Intake.getinstance();
  private  Shooter shooter = Shooter.getinstance();
  private Chassis chassis = Chassis.getinstance();
  private Automation auto = Automation.getinstance();
  private ShooterTransportation shooterTransportation = ShooterTransportation.getinstance();

  public static XboxController OperatingJoystick = new XboxController(2);
  public static Joystick leftJoystick = new Joystick(0);
  public static Joystick rightJoystick = new Joystick(1);

  private JoystickButton transportationControlButton = new JoystickButton(OperatingJoystick, 1);
  private JoystickButton pullIntake = new JoystickButton(OperatingJoystick, 2);
  private JoystickButton openCloseIntake = new JoystickButton(OperatingJoystick, 5);

  private static JoystickButton pushOut = new JoystickButton(OperatingJoystick, 6);
  private static JoystickButton PIDFlyWheel = new JoystickButton(OperatingJoystick, 3);
  private static JoystickButton PIDSquishMotor = new JoystickButton(OperatingJoystick, 4);
  
  private JoystickButton PIDVision = new JoystickButton(OperatingJoystick, 1);
  private JoystickButton MApath = new JoystickButton(OperatingJoystick, 2);
  private JoystickButton pathWriter = new JoystickButton(rightJoystick, 2);

  private  CommandBase shooting = new Shooting(auto);
  private  CommandBase preparationShooting = new PreparationShooting(auto);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    transportationControlButton.whileHeld(new PIDSquishMotor(shooterTransportation));
    pullIntake.whileHeld(new TransportationContorl(transportation));
    openCloseIntake.whenPressed(new IntakeOpenClose(intake));
    PIDFlyWheel.whileHeld(new IntakeAutomation(auto));
    PIDSquishMotor.whileHeld(new Shooting(auto));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {

  // return
  // }
}

