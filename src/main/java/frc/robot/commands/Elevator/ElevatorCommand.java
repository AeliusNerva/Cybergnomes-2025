// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.Elevator.ElevatorSetPosition;
//import frc.robot.Constants.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  private ElevatorSetPosition setPosition;
  private Elevator elevatorInstance;
  Timer m_timer = new Timer();


 // private Elevator. targetPoint = Elevator.ElevatorHoldPoint.NONE;
  //private final Elevator s_Elevator = RobotContainer.s_Elevator;

    // private final double speed;
    // private final double position;
  

    //private final Timer timer = new Timer();


  /** Creates a new Elevator. */
   public ElevatorCommand(ElevatorSetPosition setPosition) {
    this.setPosition = setPosition;

    elevatorInstance = Elevator.getInstance();
    addRequirements(elevatorInstance);
   }
//     //requires(RobotContainer.s_Elevator);

//    /*  if (isFinished()) {
//       cancel();
//       return;
//     }
// */
//     this.speed = speed;
//     this.position = position;
//     //this.height = height;
//   }

  //public Elevator(double targetFeet) {

  //}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorInstance.setPosition(setPosition.position);
    //s_Elevator.setHeight(Constants.Elevator.ZERO_HEIGHT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //s_Elevator.setPosition.(400);
    elevatorInstance.setPosition(setPosition.position);

  }

    //Min and Max
    //position = s_Elevator.getCanCoderPosition() - Constants.Elevator.CANCODER_MIN;
// else
//     position = Constants.Elevator.CANCODER_MAX - s_Elevator.getCanCoderPosition();



  //s_Elevator.setSpeed(position );
   // s_Elevator.setSpeed(speed);
   // s_Elevator.setPosition(Constants.Elevator.LEVEL_1_HEIGHT);
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   elevatorInstance.stopMotor();
   elevatorInstance.resetPrevPosition(setPosition.position);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //s_Elevator.setSpeed(0);
    if (elevatorInstance.isAtTargetPosition() == true || m_timer.hasElapsed(5)) {
    return true;
   }
    else
    {
     return false;
    }
  
  }
  

}
