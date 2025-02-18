// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private TalonFX fxLeftElevatorMotor;
  private TalonFX fxRightElevatorMotor;
  private TalonFXConfiguration fxElevatorConfig;

  // Elevator has 2 falcon500 motors to move the sliders of the elevator up and down

  public Elevator() {
    //Define motors
    fxLeftElevatorMotor = new TalonFX(0);
    fxRightElevatorMotor = new TalonFX(0);

    //Configure motors
    fxElevatorConfig = new TalonFXConfiguration();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
