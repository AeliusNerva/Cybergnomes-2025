// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import frc.lib.lib2013.SubsystemChecker;
//import frc.lib.SubsystemChecker.SubsystemType;

//import com.ctre.phoenix.motorcontrol


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.Elevator.LeftElevatorMotor;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
/* */
public class Elevator extends SubsystemBase {
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final Elevator s_Elevator = RobotContainer.s_Elevator;
  private static Elevator instance = null;

  public double elevatorTargetPosition = 0;
  public double elevatorPrevPosition = 0;

  private TalonFX fxLeftElevatorMotor;
  private TalonFX fxRightElevatorMotor;
  private TalonFXConfiguration fxLeftElevatorMotorConfig;
  private TalonFXConfiguration fxRightElevatorMotorConfig;

  private final PIDController elevatorController;

  //private CANcoder ElevatorCanCoder;

  public static Elevator getInstance() {
    if (instance == null) {
     // SubsystemChecker.subsystemConstructed(SubsystemType.Elevator);
      instance = new Elevator();
    }
    return instance;
  }

    public Elevator() {

      
      // Define motors
      fxLeftElevatorMotor = new TalonFX(39);
      fxRightElevatorMotor = new TalonFX(15);
  
      // Configure motors
      fxLeftElevatorMotorConfig = new TalonFXConfiguration();
      fxRightElevatorMotorConfig = new TalonFXConfiguration();

      elevatorController = new PIDController(0.2, 0, 0 );

      // groups motors together, telling it to listen to the left motor
      //fxElevatorConfig.Feedback.FeedbackRemoteSensorID = Constants.Elevator.LeftElevatorMotor.MOTOR_ID;
      //fxElevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

  
  
      // Acceleration/decelration of elevator
     /*  MotionMagicConfigs angleMotionMagic = fxElevatorMotorConfig.MotionMagic;
          angleMotionMagic.MotionMagicAcceleration = Constants.Elevator.LeftElevatorMotor.ACCELERATION;
          angleMotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.LeftElevatorMotor.MAX_SPEED;

          angleMotionMagic.MotionMagicAcceleration = Constants.Elevator.RightElevatorMotor.ACCELERATION;
          angleMotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.RightElevatorMotor.MAX_SPEED;*/

          
  
          /*Slot0Configs slot0 = fxElevatorConfig.Slot0;
          slot0.kP = Constants.Elevator.LeftElevatorMotor.KP;
          slot0.kI = Constants.Elevator.LeftElevatorMotor.KI;
          slot0.kD = Constants.Elevator.LeftElevatorMotor.KD;
          slot0.kP = Constants.Elevator.RightElevatorMotor.KP;
          slot0.kI = Constants.Elevator.RightElevatorMotor.KI;
          slot0.kD = Constants.Elevator.RightElevatorMotor.KD;*/

          // Apply configurations
          //fxLeftElevatorMotor.motorout
          fxLeftElevatorMotor.getConfigurator().apply(fxLeftElevatorMotorConfig);
          fxRightElevatorMotor.getConfigurator().apply(fxRightElevatorMotorConfig);

          // Make motors stop when not accelerating
          fxLeftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
          fxRightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        
        }
          
         public double getPosition() {
          return fxLeftElevatorMotor.getPosition().getValueAsDouble();
      }

      // @Override
      // public void initialize() {
      //  fxRightElevatorMotor.follow(fxLeftElevatorMotor);
       
      //    }
  

     @Override
     public void periodic() {
      SmartDashboard.putNumber("Elevator position", fxLeftElevatorMotor.getPosition().getValueAsDouble());
      //SmartDashboard.putNumber("elevator motor ", fxLeftElevatorMotor.getPosition().getValue());

      
        }

  
    public void setSpeed(double speedPercent) {
      fxLeftElevatorMotor.set(speedPercent);
      fxRightElevatorMotor.set(-speedPercent);
    }

     public void setPosition(double position) {
      fxLeftElevatorMotor.set(elevatorController.calculate(getPosition(), position));
      fxRightElevatorMotor.set(-elevatorController.calculate(getPosition(), position));
     }

     public void setTargetPosition( double targetPosition) {
      elevatorTargetPosition = targetPosition;
     }

     public void resetPrevPosition( double prevPosition) {
      elevatorPrevPosition = prevPosition;
     }

     public boolean isAtTargetPosition() {
      if ( Math.abs(elevatorTargetPosition - s_Elevator.getPosition()) < Constants.Elevator.ELEVATOR_POS)
        return true;
      else
        return false;
     }

    /*  public boolean isAtTargetPosition() {
      if( Math.abs(elevatorTargetPosition - s_Elevator.getPosition()) < Constants.Elevator.ELEVATOR_POS);
     }*/

     public void stopMotor() {
      s_Elevator.stopMotor();
     }
    /*public void setHeight(double MOVE_POSITION) {
      fxLeftElevatorMotor.setControl(m_mmReq.withPosition(MOVE_POSITION).withSlot(0));
      fxRightElevatorMotor.setControl(m_mmReq.withPosition(-MOVE_POSITION).withSlot(0));
      
    }*/
    //!!!!!!!!!!!!!

  //     public void setPositionRotation(double rotation) {
  //     if (rotation > Constants.Elevator.ELEVATOR_MAX || rotation < Constants.Elevator.ELEVATOR_MIN) {
  //         System.err.println(rotation + " is not a valid shaft rotation (max: 100, min: 0)");
  //         return;
  //     }
  //     fxLeftElevatorMotor.setControl(m_mmReq.withPosition(rotation).withSlot(0));
  //     fxRightElevatorMotor.setControl(m_mmReq.withPosition(rotation).withSlot(0));
  // }

//   public void setPosition(double degrees) {
//     double targetEncoderValue = (55 - degrees) * 0.78;

//     if (targetEncoderValue < Constants.Elevator.ELEVATOR_MIN)
//         targetEncoderValue = Constants.Elevator.ELEVATOR_MIN;

//     else if (targetEncoderValue > Constants.Elevator.ELEVATOR_MAX)
//         targetEncoderValue = Constants.Elevator.ELEVATOR_MAX;

//     setPositionRotation(targetEncoderValue);
// }
  //   public void setZero() {
  // s_Elevator.setPositionRotation(0);
  // }

 /*  public enum ElevatorState{
    NONE,
    LEVEL_1_HEIGHT,
    LEVEL_2_HEIGHT,
    LEVEL_3_HEIGHT,
    LEVEL_4_HEIGHT
  }*/

  /*public double getCanCoderPosition() {
    return ElevatorCanCoder.getPosition().getValue();
}*/

    //public void store.setHeight(){
      
    //}

  }
  
   /*  public void goToElevatorStore() {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = Constants.Elevator.StoreHeight;
    
  /* 
    public void goToElevatorL2() {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = Constants.Elevator.L2Height;
      mPeriodicIO.state = ElevatorState.L2;
  }
  
    public void goToElevatorL3(){
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = Constants.Elevator.L3Height;
      mPeriodicIO.state = ElevatorState.L3;
  }

  public void goToElevatorL4() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.L4Height;
    mPeriodicIO.state = ElevatorState.L4;
  }*/

//}

