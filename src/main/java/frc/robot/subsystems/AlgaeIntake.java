// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */

  private TalonFX fxAlgaeIntakeMotor;
  private TalonFXConfiguration fxConfig;
  
  // one falcon 500 motor for the roller intake to run

  public AlgaeIntake() {
    fxAlgaeIntakeMotor = new TalonFX(0);
    fxConfig = new TalonFXConfiguration();
     fxAlgaeIntakeMotor.getConfigurator().apply(fxConfig);
     
     fxAlgaeIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
   // SmartDashboard.putNumber("Roller Motor Angle", fxAlgaeIntakeMotor.getPosition().getValue());
    // This method will be called once per scheduler run
  }
}
