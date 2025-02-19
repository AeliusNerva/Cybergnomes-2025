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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  private TalonFX fxLeftElevatorMotor;
  private TalonFX fxRightElevatorMotor;
  private TalonFXConfiguration fxElevatorConfig;

  // Elevator has 2 falcon500 motors to move the sliders of the elevator up and
  // down

  public Elevator() {
    // Define motors
    fxLeftElevatorMotor = new TalonFX(0);
    fxRightElevatorMotor = new TalonFX(0);

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
    // This method will be called once per scheduler run
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
