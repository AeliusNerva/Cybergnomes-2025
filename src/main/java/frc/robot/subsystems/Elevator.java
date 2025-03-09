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


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
/* */
public class Elevator extends SubsystemBase {
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  private TalonFX fxLeftElevatorMotor;
  private TalonFX fxRightElevatorMotor;
  private TalonFXConfiguration fxElevatorConfig;

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
        
        }
          
      

     @Override
     public void periodic() {
      
        }

  
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

