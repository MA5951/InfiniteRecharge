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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Trigger.PreShootingTrigger;
import frc.robot.Trigger.ShootingTriggger;
import frc.robot.commands.Automation.IntakeAutomation;
import frc.robot.commands.Automation.Shooting;
import frc.robot.commands.Autonomous.EnemyRoullete;
import frc.robot.commands.Autonomous.roulletePath;
import frc.robot.commands.Autonomous.roulletePath1;
import frc.robot.commands.Autonomous.shootanddrive;
import frc.robot.commands.Autonomous.standert1;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.commands.Chassis.PIDVision;
import frc.robot.commands.Chassis.PIDVisionFeeder;
import frc.robot.commands.Elevator.ElevatorMotorControl;
import frc.robot.commands.Elevator.OpenAndClosePiston;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Balance;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Transportation.*;
import frc.robot.subsystems.Transportation;
import frc.robot.commands.Intake.IntakClose;
import frc.robot.commands.Intake.IntakePullPush;
import frc.robot.commands.Intake.OpenIntake;
import frc.robot.commands.Roulette.RoulletControlPiston;
import frc.robot.commands.Roulette.roundTwoRoulettePID;
import frc.robot.commands.Shooter.PIDFlyWheel;
import frc.robot.commands.Shooter.PIDFlyWheelAutonumos;
import frc.robot.commands.ShooterTransportation.PIDSquishMotor;
import frc.robot.commands.ShooterTransportation.reversSquish;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limlight;
import frc.robot.subsystems.Roulette;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Automation.CancelAllMotors;
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

  private Intake intake = Intake.getinstance();
  private Transportation transportation = Transportation.getinstance();
  private ShooterTransportation shooterTransportation = ShooterTransportation.getinstance();
  private Shooter shooter = Shooter.getinstance();
  private Automation auto = Automation.getinstance();
  private Elevator elevator = Elevator.getinstance();
  private Roulette roulette = Roulette.getinstance();

  public static XboxController OperatingJoystick = new XboxController(2);
  public static Joystick leftJoystick = new Joystick(0);
  public static Joystick rightJoystick = new Joystick(1);

  private JoystickButton OpenIntake = new JoystickButton(OperatingJoystick, 6);

  private JoystickButton PIDVision = new JoystickButton(leftJoystick, 5);
  private JoystickButton PIDVision1 = new JoystickButton(rightJoystick, 5);

  private JoystickButton PIDVision2 = new JoystickButton(leftJoystick, 3);
  private JoystickButton PIDVision3 = new JoystickButton(rightJoystick, 3);

  private JoystickButton CloseIntake = new JoystickButton(OperatingJoystick, 5);
  private ShootingTriggger Shoot = new ShootingTriggger();
  private JoystickButton RouletteControl = new JoystickButton(OperatingJoystick, 3);

  private JoystickButton CancelAllMotors = new JoystickButton(OperatingJoystick, 10);
  private JoystickButton CancelAllMotorsTwo = new JoystickButton(OperatingJoystick, 9);

  private JoystickButton RouletteAutomation = new JoystickButton(OperatingJoystick, 2);
  private JoystickButton IntakeAutomation = new JoystickButton(OperatingJoystick, 1);
  private JoystickButton transportations = new JoystickButton(OperatingJoystick, 4);
  private PreShootingTrigger preShootingTrigger = new PreShootingTrigger();

  private POVButton PistonElevatorOpen = new POVButton(OperatingJoystick, 0);
  private POVButton PistonElevatorClose = new POVButton(OperatingJoystick, 180);

  private POVButton RouletteRight = new POVButton(OperatingJoystick, 90);
  private POVButton RouletteLeft = new POVButton(OperatingJoystick, 270);

  private JoystickButton reversall = new JoystickButton(OperatingJoystick, 8);

  EnemyRoullete EnemyroulletePath = new EnemyRoullete(Autonomous.getInstance());
  roulletePath roulletePath = new roulletePath(Autonomous.getInstance());
  shootanddrive shootandDrive = new shootanddrive();
  standert1 standert = new standert1();
  roulletePath1 roulletepath1 = new roulletePath1(Autonomous.getInstance());

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
    PistonElevatorOpen.whenPressed(new OpenAndClosePiston(elevator, true));
    PistonElevatorClose.whenPressed(new OpenAndClosePiston(elevator, false));

    OpenIntake.whenPressed(new OpenIntake(intake));
    CloseIntake.whenPressed(new IntakClose(intake));
    transportations.whileHeld(new TransportationContorl(transportation));
    transportations.whileHeld(new PIDSquishMotor(shooterTransportation));
    Shoot.whileActiveContinuous(new Shooting(auto, false));
    preShootingTrigger.whileActiveContinuous(new PIDFlyWheelAutonumos(shooter, 3500));

    PIDVision3.whileHeld(new PIDVisionFeeder(Limlight.getInstance()));
    PIDVision2.whileHeld(new PIDVisionFeeder(Limlight.getInstance()));

    PIDVision.whileHeld(new PIDVision(0, 0.1, Limlight.getInstance()));
    PIDVision1.whileHeld(new PIDVision(0, 0.1, Limlight.getInstance()));
    // PIDVision2.whileHeld(new PIDVision(0, 0.1, Limlight.getInstance()));

    RouletteControl.whileHeld(new IntakePullPush(0.5, intake));

    reversall.whileHeld(new reversTransportationContorl());
    reversall.whileHeld(new reversSquish());

    RouletteLeft.whileActiveContinuous(new roundTwoRoulettePID(-0.5, roulette));
    RouletteRight.whileActiveContinuous(new roundTwoRoulettePID(0.5, roulette));

    IntakeAutomation.whileHeld(new IntakeAutomation(auto));

    RouletteAutomation.whenPressed(new RoulletControlPiston());

    CancelAllMotors.whenPressed(new CancelAllMotors(auto));
    CancelAllMotorsTwo.whenPressed(new CancelAllMotors(auto));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (SmartDashboard.getNumber("auto", 1) == 1) {
      return roulletePath;
    } else if (SmartDashboard.getNumber("auto", 1) == 2) {
      return EnemyroulletePath;
    } else if (SmartDashboard.getNumber("auto", 1) == 3) {
      return shootandDrive;
    } else if (SmartDashboard.getNumber("auto", 1) == 4) {
      return standert;
    } else {
      return roulletepath1;
    }
  }
}
