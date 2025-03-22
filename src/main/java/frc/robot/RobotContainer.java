// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.Algae.IntakeCommand;
import frc.robot.commands.Algae.IntakeWheelsCommand;
import frc.robot.commands.Claw.PivotCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Elevator.ReverseElevatorCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.Algae.IntakeCommand;
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
    /* Controllers */
    // Main Driver Controller
    public static final Joystick driver = new Joystick(0);
    private final GenericHID driver_hid = new GenericHID(0);

    // Side Driver Controller
    private final Joystick coDriver = new Joystick(1);
    private final GenericHID coDriver_hid = new GenericHID(1);
 
  // Buttons
  private final JoystickButton b_robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
 
  // Subsytems
  public static final Swerve s_Swerve = new Swerve();
  public static final Elevator s_Elevator = new Elevator();
  public static final Timer Timer = new Timer();
  public static final CoralIntake s_CoralIntake = new CoralIntake();
  public static final Pivot s_Pivot = new Pivot();
  public static final AlgaeIntake s_AlgaeIntakeWheels = new AlgaeIntake();
  public static final AlgaeIntake s_AlgaeIntake1 = new AlgaeIntake();
  public static final AlgaeIntake s_AlgaeIntake2 = new AlgaeIntake();


  /* Sendable Chooser and Autonomus Commands */
  private static SendableChooser<Command> autoChooser;

  // Drive controls
  public static final int translationAxis = PS4Controller.Axis.kLeftY.value;
  public static final int strafeAxis = PS4Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS4Controller.Axis.kRightX.value;
  private final int rotationTargetAxis = 3; // RightY: 3;

  
    // spin elevator motors (2 motors) to move elevator (upwards)
    //private final JoystickButton b_reverseElevator = new JoystickButton(driver, PS4Controller.Button.kL1.value);

    // Spin elevator motors (2 motors) backward (down)
    //private final JoystickButton b_spinElevator = new JoystickButton(driver, PS4Controller.Button.k.value);


    // ***ELEVATOR BUTTONS!!!!***//
        //DRIVER
    private final JoystickButton b_Level1 = new JoystickButton(driver,  PS4Controller.Button.kCross.value);
    private final JoystickButton b_Level2 = new JoystickButton(driver,  PS4Controller.Button.kSquare.value);
    private final JoystickButton b_Level3 = new JoystickButton(driver,  PS4Controller.Button.kCircle.value);
    private final JoystickButton b_Level4 = new JoystickButton(driver,  PS4Controller.Button.kTriangle.value);
    private final JoystickButton b_Zero = new JoystickButton(driver,  PS4Controller.Button.kShare.value);
        //CO DRIVER
    private final JoystickButton b_Level1_coDriver = new JoystickButton(coDriver,  PS4Controller.Button.kCross.value);
    private final JoystickButton b_Level2_coDriver = new JoystickButton(coDriver,  PS4Controller.Button.kSquare.value);
    private final JoystickButton b_Level3_coDriver = new JoystickButton(coDriver,  PS4Controller.Button.kCircle.value);
    private final JoystickButton b_Level4_coDriver = new JoystickButton(coDriver,  PS4Controller.Button.kTriangle.value);
    private final JoystickButton b_Zero_coDriver = new JoystickButton(coDriver,  PS4Controller.Button.kShare.value);

    //*** CORAL PUSHER/FUNNEL INTAKE ***//
        //DRIVER
    private final JoystickButton b_coralPusherBlock = new JoystickButton(driver, PS4Controller.Button.kR2.value);
        //CODRIVER
    private final JoystickButton b_coralPusherBlock_coDriver = new JoystickButton(coDriver, PS4Controller.Button.kR2.value);

    //*** CLAW ***//
        //DRIVER
    private final JoystickButton b_ClawOpenBlock = new JoystickButton(driver,PS4Controller.Button.kL1.value); // claw to open
    private final POVButton b_PivotUp = new POVButton(driver_hid, 0); // PIVOT to angle up while coral is open (pushed into claw)
    private final JoystickButton b_ClawCloseBlock = new JoystickButton(driver,PS4Controller.Button.kL2.value); // claw to CLOSE
    private final POVButton b_PivotOut = new POVButton(driver_hid, 270); // PIVOT to angle OUT while coral is CLOSED (pushed into claw)
      //CO DRIVER
    private final POVButton b_ClawOpenBlock_coDriver = new POVButton(coDriver_hid, 0); // claw to open
    private final POVButton b_PivotUp_coDriver = new POVButton(coDriver_hid, 0); // PIVOT to angle up while coral is open (pushed into claw)
    private final POVButton b_ClawCloseBlock_coDriver = new POVButton(coDriver_hid, 270); // claw to CLOSE
    private final POVButton b_PivotOut_coDriver = new POVButton(coDriver_hid, 270); // PIVOT to angle OUT while coral is CLOSED (pushed into claw)

    //*** ALGAE INTAKE ***/
        //DRIVER
   /*  private final JoystickButton b_AlgaeIntake1Block = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton b_AlgaeIntake2Block = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton b_AlgaeIntakeWheels = new JoystickButton(driver, PS4Controller.Button.kL1.value);
        //CO DRIVER
    private final JoystickButton b_AlgaeIntake1Block_coDriver = new JoystickButton(coDriver, PS4Controller.Button.kL1.value);
    private final JoystickButton b_AlgaeIntake2Block_coDriver = new JoystickButton(coDriver, PS4Controller.Button.kL1.value);
    private final JoystickButton b_AlgaeIntakeWheels_coDriver = new JoystickButton(coDriver, PS4Controller.Button.kL1.value);
   */


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve.configureAutoBuilder();
    //configurePathPlannerCommands();
    boolean isCompetition = true;

    /* Setup */
    setUpSwerveController();
    configureLimelight(Constants.Limelight.Right.NAME);
    configureLimelight(Constants.Limelight.Left.NAME);
    configureBindings();
    preparePneumatics();
    configureAutoChooser();

  }

  public static final PneumaticsHandler h_pneumatics = new PneumaticsHandler();

  private void preparePneumatics() {
    h_pneumatics.setCoralPusherSolenoid(false);
    h_pneumatics.setClawSolenoid(false);
 }

 // Only for main driver
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
  private void configureAutoChooser() {
    // Autonomous Sendable Chooser
    autoChooser = AutoBuilder.buildAutoChooser("StraightAuto");

    SmartDashboard.putData("Selected Auto", autoChooser);
  }

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

       /* Elevator Buttons */
          //Driver
       b_Level1.onTrue(new ElevatorCommand(Constants.Elevator.ElevatorSetPosition.LEVEL_1_HEIGHT));
       b_Level2.onTrue(new ElevatorCommand(Constants.Elevator.ElevatorSetPosition.LEVEL_2_HEIGHT));
       b_Level3.onTrue(new ElevatorCommand(Constants.Elevator.ElevatorSetPosition.LEVEL_3_HEIGHT));
       b_Level4.onTrue(new ElevatorCommand(Constants.Elevator.ElevatorSetPosition.LEVEL_4_HEIGHT));
       b_Zero.onTrue(new ElevatorCommand(Constants.Elevator.ElevatorSetPosition.ZERO_HEIGHT));
          //Co driver
       b_Level1_coDriver.onTrue(new ElevatorCommand(Constants.Elevator.ElevatorSetPosition.LEVEL_1_HEIGHT));
       b_Level2_coDriver.onTrue(new ElevatorCommand(Constants.Elevator.ElevatorSetPosition.LEVEL_2_HEIGHT));
       b_Level3_coDriver.onTrue(new ElevatorCommand(Constants.Elevator.ElevatorSetPosition.LEVEL_3_HEIGHT));
       b_Level4_coDriver.onTrue(new ElevatorCommand(Constants.Elevator.ElevatorSetPosition.LEVEL_4_HEIGHT));
       b_Zero_coDriver.onTrue(new ElevatorCommand(Constants.Elevator.ElevatorSetPosition.ZERO_HEIGHT));

       //b_spinElevator.whileTrue(new InstantCommand(() -> s_Elevator.setPosition(-400)));
       //b_spinElevator.onFalse(new ElevatorCommand(0));
       //b_spinElevator.whileTrue(new ElevatorCommand(0.3));

       //b_reverseElevator.whileTrue(new InstantCommand(() -> s_Elevator.setPosition(800)));
       //b_reverseElevator.onTrue(new ElevatorCommand(-0.3, 2));
       //b_reverseElevator.onFalse(new ReverseElevatorCommand(0));

       /*Pivot button (square) */
       //b_Pivot.onTrue(new PivotCommand(Constants.Claw.PivotSetPosition.UP));

       /* coral pusher (funnel intake) */
          //Driver
       b_coralPusherBlock.onTrue(new InstantCommand(() -> h_pneumatics.setCoralPusherSolenoid(true)));
       b_coralPusherBlock.onFalse(new InstantCommand(() -> h_pneumatics.setCoralPusherSolenoid(false)));
          //Co driver
       b_coralPusherBlock_coDriver.onTrue(new InstantCommand(() -> h_pneumatics.setCoralPusherSolenoid(true)));
       b_coralPusherBlock_coDriver.onFalse(new InstantCommand(() -> h_pneumatics.setCoralPusherSolenoid(false)));

       /*Algae intake */
      /*  b_AlgaeIntakeWheels.whileTrue(new IntakeWheelsCommand(0.5));

       b_AlgaeIntake1Block.onTrue(new InstantCommand(() -> h_pneumatics.setAlgaeIntake1Solenoid(true)));
       b_AlgaeIntake2Block.onTrue(new InstantCommand(() -> h_pneumatics.setAlgaeIntake2Solenoid(true)));

       b_AlgaeIntake1Block.onFalse(new InstantCommand(() -> h_pneumatics.setAlgaeIntake1Solenoid(false)));
       b_AlgaeIntake2Block.onFalse(new InstantCommand(() -> h_pneumatics.setAlgaeIntake2Solenoid(false)));
      */
       /* claw grab and release */
          //Driver
       b_ClawOpenBlock.onTrue(new InstantCommand(() -> h_pneumatics.setClawSolenoid(true)));//open
       b_ClawCloseBlock.onFalse(new InstantCommand(() -> h_pneumatics.setClawSolenoid(false)));//closed
          //Co driver
       b_ClawOpenBlock_coDriver.onTrue(new InstantCommand(() -> h_pneumatics.setClawSolenoid(true)));//open
       b_ClawCloseBlock_coDriver.onFalse(new InstantCommand(() -> h_pneumatics.setClawSolenoid(false)));//closed

       /* pivot */
          //Driver
       b_PivotUp.onTrue(new PivotCommand(Constants.Claw.PivotSetPosition.UP)); 
       b_PivotOut.onTrue(new PivotCommand(Constants.Claw.PivotSetPosition.OUT));
          //Co driver
       b_PivotUp_coDriver.onTrue(new PivotCommand(Constants.Claw.PivotSetPosition.UP)); 
       b_PivotOut_coDriver.onTrue(new PivotCommand(Constants.Claw.PivotSetPosition.OUT));


  }

  private void stopMotors() {
    s_Elevator.setSpeed(0);
    s_Pivot.setSpeed(0);
  }

 // private void preparePneumatics() {

 // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
    //return new PathPlannerAuto("Test Path");
  }
}
