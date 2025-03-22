// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.Constants.Claw.ClawPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants.Claw.PivotMotor;
import frc.robot.Constants.Claw.PivotSetPosition;
import frc.robot.Constants.Elevator.ElevatorSetPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotCommand extends Command {
  private PivotSetPosition setPivotPosition;
  private Pivot pivotInstance;
  Timer m_timer = new Timer();
  
  private final Pivot s_Pivot = RobotContainer.s_Pivot;

   public PivotCommand(PivotSetPosition setPivotPosition) {
    this.setPivotPosition = setPivotPosition;

    pivotInstance = Pivot.getInstance();
    addRequirements(pivotInstance);

  }

  // Called when the command is initially scheduled.
  @Override //initially zeros (resets) the motor
  public void initialize() {
    pivotInstance.setPosition(setPivotPosition.pivotPosition); 
    //s_Pivot.setZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override //moves the claw to 90 degrees (target angle)
  public void execute() {
    //s_Pivot.settargetAngle(8.77);
    pivotInstance.setPosition(setPivotPosition.pivotPosition);
  }

  // Called once the command ends or is interrupted.
  @Override // Stops the claw when the command is ended 
  public void end(boolean interrupted) {
    pivotInstance.stopMotor();
    pivotInstance.resetPrevPosition(setPivotPosition.pivotPosition);  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pivotInstance.isAtTargetPosition() == true || m_timer.hasElapsed(5)) {
      return true;
     }
      else
      {
       return false;
      }
  }
      
}
