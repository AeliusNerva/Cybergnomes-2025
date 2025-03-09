// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.Constants.Claw.ClawPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants.Claw.ClawPivotMotor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotCommand extends Command {
  /** Creates a new ClawPivotCommand. */
  private final Pivot s_Pivot = RobotContainer.s_Pivot;

  private double targetAngle;// Target Angle for the claw to pivot in degrees

   public PivotCommand(double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetAngle = targetAngle;
    addRequirements(s_Pivot);
  }

  // Called when the command is initially scheduled.
  @Override //initially zeros (resets) the motor
  public void initialize() {
    s_Pivot.setZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override //moves the claw to 90 degrees (target angle)
  public void execute() {
    s_Pivot.setPivotAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override // Stops the claw when the command is ended 
  public void end(boolean interrupted) {
    s_Pivot.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //COME BACK TO THIS!!!!!!!!
   // return Math.abs(s_Pivot.getPivotPosition() - targetAngle) < Constants.Claw.ClawPivotMotor.PIVOT_ANGLE_THRESHOLD;

  }
}
