// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveCommand;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Elevator.ReverseElevatorCommand;
import frc.robot.commands.CoralIntakeCommand;
//import frc.robot.generated.TunerConstants;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Driver controller
  public static final Joystick driver = new Joystick(0);
 
  // Buttons
  private final JoystickButton b_robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
 
  // Subsytems
  public static final Swerve s_Swerve = new Swerve();
  public static final Elevator s_Elevator = new Elevator();
  public static final Timer shooterTimer = new Timer();
  public static final CoralIntake s_CoralIntake = new CoralIntake();
  public static final Pivot s_Pivot = new Pivot();

  /* Sendable Chooser and Autonomus Commands */
  private static SendableChooser<Command> autoChooser;

  // Drive controls
  public static final int translationAxis = PS4Controller.Axis.kLeftY.value;
  public static final int strafeAxis = PS4Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS4Controller.Axis.kRightX.value;
  private final int rotationTargetAxis = 3; // RightY: 3;

  
    // spin elevator motors (2 motors) to move elevator (upwards)
    private final JoystickButton b_spinElevator = new JoystickButton(driver, PS4Controller.Button.kL2.value);

    // Spin elevator motors (2 motors) backward (down)
    private final JoystickButton b_reverseElevator = new JoystickButton(driver, PS4Controller.Button.kL1.value);

    private final JoystickButton b_clawPivot = new JoystickButton(driver, PS4Controller.Button.kSquare.value);

   


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
   // s_Swerve.configureAutoBuilder();

    /* Setup */
    setUpSwerveController();
    configureLimelight(Constants.Limelight.Right.NAME);
    configureLimelight(Constants.Limelight.Left.NAME);
    configureBindings();
    //configureAutoChooser();



  }

  public static final PneumaticsHandler h_pneumatics = new PneumaticsHandler();

  private void preparePneumatics() {
    h_pneumatics.setCoralPusherSolenoid(false);
 }

  private void setUpSwerveController() {
    s_Swerve.setDefaultCommand(new SwerveCommand(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        () -> -driver.getRawAxis(rotationTargetAxis),
        () -> b_robotCentric.getAsBoolean()));
  }

  private void configureLimelight(String limelightName) {
    LimelightHelpers.setLEDMode_ForceOn(limelightName);
    LimelightHelpers.setStreamMode_Standard(limelightName);
    LimelightHelpers.setCameraMode_Processor(limelightName);
}
  //private void configureAutoChooser() {
    // Autonomous Sendable Chooser
   // autoChooser = AutoBuilder.buildAutoChooser("Middle Side - 3 Note");

   // SmartDashboard.putData("Selected Auto", autoChooser);
 // }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

       /* Intake Buttons */

       b_spinElevator.whileTrue(new ElevatorCommand());
       b_reverseElevator.whileTrue(new ReverseElevatorCommand());

  }

  private void stopMotors() {
    s_Elevator.setSpeed(0);
    //s_ClawPivot.setSpeed(0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  //  return autoChooser.getSelected();
  //}
}
