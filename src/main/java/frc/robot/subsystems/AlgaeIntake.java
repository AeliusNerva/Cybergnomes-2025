// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  /** Creates a new AlgaeIntake. */

  private TalonFX fxAlgaeIntakeMotor;
  private TalonFXConfiguration fxConfig;
  
  // one falcon 500 motor for the roller intake to run

  public AlgaeIntake() {
    fxAlgaeIntakeMotor = new TalonFX(6);
    fxConfig = new TalonFXConfiguration();
     fxAlgaeIntakeMotor.getConfigurator().apply(fxConfig);
     
     fxAlgaeIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void periodic() {}
   // SmartDashboard.putNumber("Roller Motor Angle", fxAlgaeIntakeMotor.getPosition().getValue());
    // This method will be called once per scheduler run

    public void setSpeed(double speedPercent) {
      fxAlgaeIntakeMotor.set(speedPercent);
    }

    /* public void setHeight(double position) {
      fxAlgaeIntakeMotor.setControl(m_mmReq.withPosition(position).withSlot(0));*/


  }

