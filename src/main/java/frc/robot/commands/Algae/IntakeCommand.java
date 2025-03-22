// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PneumaticsHandler;
import frc.robot.RobotContainer;
import frc.robot.Constants.Algae;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  /** Creates a new Intake. */
  private final AlgaeIntake s_AlgaeIntake1 = RobotContainer.s_AlgaeIntake1;
  private final AlgaeIntake s_AlgaeIntake2 = RobotContainer.s_AlgaeIntake2;
  //private final Algae s_AlgaeIntake1Block = RobotContainer.AlgaeIntake;
  //private final Algae s_AlgaeIntake2Block = RobotContainer.s_AlgaeIntake;

  private final double speed;
  private final Timer timer = new Timer();
  private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;
  public IntakeCommand(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    h_pneumatics.setAlgaeIntake1Solenoid(false);
    h_pneumatics.setAlgaeIntake2Solenoid(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //s_AlgaeIntake.setSpeed(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    h_pneumatics.setAlgaeIntake1Solenoid(true);
    h_pneumatics.setAlgaeIntake2Solenoid(true);
    //s_AlgaeIntake.setSpeed(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
