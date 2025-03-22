// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PneumaticsHandler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntakeCommand extends Command {
  /** Creates a new CoralIntake. */

  //Pushing pnuematic mechanism that will push the coral into the claw

  private final CoralIntake s_coralPusherBlock = RobotContainer.s_CoralIntake;
  private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    private final double speed;
    private final Timer timer = new Timer();


    public CoralIntakeCommand(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    // Unlock climber
    h_pneumatics.setCoralPusherSolenoid(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    h_pneumatics.setCoralPusherSolenoid(true);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
    return false;
  }
}
