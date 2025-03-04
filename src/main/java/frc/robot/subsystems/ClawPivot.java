// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawPivot extends SubsystemBase {
  /** Creates a new ClawPivot. */

  private TalonFX fxPivotMotor;
  private TalonFXConfiguration fxConfig;

  public ClawPivot() {

     fxPivotMotor = new TalonFX(0); //FILL IN!!!!!
     fxConfig = new TalonFXConfiguration();
     fxPivotMotor.getConfigurator().apply(fxConfig);
     
  }

    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
