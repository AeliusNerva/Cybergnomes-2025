// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
/* */
public class Elevator extends SubsystemBase {
  //private static final String ElevatorState = null;

   
   // private PeriodicIO mPeriodicIO;
    //public static ElevatorState m_ElevatorRequestedState;
   // private double desiredPosition = 0;
  

     public enum ElevatorState {
      StateInit,
      NONE,
      STORE,
      L1,
      L2,
      L3,
      L4,
      StateMoveToRequestedState,
    }

    public static ElevatorState m_ElevatorCurrentState;
    public static ElevatorState m_ElevatorRequestedState;

    // Elevator has 2 falcon500 motors to move the sliders of the elevator up and
    // down
    private TalonFX fxLeftElevatorMotor;
    private TalonFX fxRightElevatorMotor;
    private TalonFXConfiguration fxElevatorConfig;

    MotionMagicVoltage request;

    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

    public static final double kElevatorErrorTolerance = 0.5;

    private double desiredPosition = 0;

  //            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
  public static GenericEntry motorVelo = Shuffleboard.getTab("Elevator").add("Velocity", 0.0).getEntry();
  public static GenericEntry desiredPositionLog = Shuffleboard.getTab("Elevator").add("position", 0).getEntry();
  public static GenericEntry motorPosition = Shuffleboard.getTab("Elevator").add("position2", 0.0).getEntry();


    /*private static class PeriodicIO {
      double elevator_target = 0.0;
      double elevator_power = 0.0;
}*/
    public Elevator() {

      
      // Define motors
      fxLeftElevatorMotor = new TalonFX(5);
      fxRightElevatorMotor = new TalonFX(15);
  
      // Configure motors
      fxElevatorConfig = new TalonFXConfiguration();
  
      fxLeftElevatorMotor.getConfigurator().apply(fxElevatorConfig);
  
      fxLeftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  
      fxRightElevatorMotor.getConfigurator().apply(fxElevatorConfig);
  
      fxRightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  
      // Acceleration/decelration of elevator
      MotionMagicConfigs angleMotionMagic = fxElevatorConfig.MotionMagic;
          angleMotionMagic.MotionMagicAcceleration = Constants.Elevator.LeftElevatorMotor.ACCELERATION;
          angleMotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.LeftElevatorMotor.MAX_SPEED;
  
          Slot0Configs slot0 = fxElevatorConfig.Slot0;
          slot0.kP = Constants.Elevator.LeftElevatorMotor.KP;
          slot0.kI = Constants.Elevator.LeftElevatorMotor.KI;
          slot0.kD = Constants.Elevator.LeftElevatorMotor.KD;
  
        
          angleMotionMagic.MotionMagicAcceleration = Constants.Elevator.RightElevatorMotor.ACCELERATION;
          angleMotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.RightElevatorMotor.MAX_SPEED;

          fxLeftElevatorMotor.getConfigurator().apply(fxElevatorConfig);
          // m_elevatorTalon2.getConfigurator().apply(talonFXConfigs);
          m_ElevatorCurrentState = ElevatorState.StateInit;
          m_ElevatorRequestedState = ElevatorState.StateInit;
      
          // if we design the robot with a proper resting position in mind
          // this should be the only initilization necessary
          // no firstTime2 :)
          fxLeftElevatorMotor.setPosition(0);
        
        }
          
    

     @Override
     public void periodic() {
      switch (m_ElevatorRequestedState) {
  
        case NONE:
          desiredPosition = 0; //FILL IN
          break;
        case STORE:
          desiredPosition = 0;
          break;
        case L1:
          desiredPosition = 0;
          break;
        case L2:
          desiredPosition = 1;
          break;
        case L3:
          desiredPosition = 22.75;
          break;
        case L4:
          desiredPosition = 27.625-.25;
          break;
        default:
          break;
        }

       // desiredPosition*=kELevatorInchesToOutput;
        runControlLoop();
        if (getError() < kElevatorErrorTolerance)
        m_ElevatorCurrentState = m_ElevatorRequestedState;
      else
        m_ElevatorCurrentState = ElevatorState.StateMoveToRequestedState;
     }

     public void runControlLoop() {
      fxLeftElevatorMotor.setControl(request.withPosition(desiredPosition));
      motorVelo.setDouble(fxLeftElevatorMotor.getVelocity().getValueAsDouble());
      desiredPositionLog.setDouble(fxLeftElevatorMotor.getPosition().getValueAsDouble());

      fxRightElevatorMotor.setControl(request.withPosition(desiredPosition));
      motorVelo.setDouble(fxRightElevatorMotor.getVelocity().getValueAsDouble());
      desiredPositionLog.setDouble(fxRightElevatorMotor.getPosition().getValueAsDouble());

     }

     private double getPosition(){
      return fxLeftElevatorMotor.getPosition().getValueAsDouble();
      //return fxRightElevatorMotor.getPosition().getValueAsDouble();

     }

     public double getError() {
      return Math.abs(getPosition() - desiredPosition);
     }

     //'setter' method
     public void requestState(ElevatorState requestedState){
      m_ElevatorRequestedState = requestedState;

     }

     // 'getter' method
     public ElevatorState getCurrentState() {
      return m_ElevatorCurrentState;
    }

    public ElevatorState getRequestedState(){

      return m_ElevatorRequestedState;
    }
    
    /*@Override
    public void periodic() {
      // This method will be called once per scheduler run
    }*/
  
    public void setSpeed(double speedPercent) {
      fxLeftElevatorMotor.set(speedPercent);
      fxRightElevatorMotor.set(-speedPercent);
    }
  
    public void setHeight(double position) {
      fxLeftElevatorMotor.setControl(m_mmReq.withPosition(position).withSlot(0));
      fxRightElevatorMotor.setControl(m_mmReq.withPosition(-position).withSlot(0));
    }
  }
    /*public void goToElevatorStore() {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = Constants.Elevator.StoreHeight;
      mPeriodicIO.state = ElevatorState.L2;
    }
  
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

