// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseElevatorCommand extends Command {
  /** Creates a new ReverseElevator. */

  private final Elevator s_Elevator = RobotContainer.s_Elevator;
  private final double speed;
  //private final double position;

  public ReverseElevatorCommand(double speed) {
    //requires(RobotContainer.s_Elevator);

   
  
   /*  if (isFinished()) {
      cancel();
      return;
    }
*/
    this.speed = speed;
    //this.position = position;
    //this.height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      s_Elevator.setSpeed(speed);
      
     // s_Elevator.setHeight(Constants.Elevator.MOVE_POSITION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
