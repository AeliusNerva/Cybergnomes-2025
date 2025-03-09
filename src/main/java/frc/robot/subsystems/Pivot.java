// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Claw.ClawPivotMotor;
import frc.robot.subsystems.Pivot;

public class Pivot extends SubsystemBase {
  /** Creates a new ClawPivot. */

  private TalonFX fxPivotMotor;
  private TalonFXConfiguration fxPivotConfig;
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  public Pivot() {
    // Define motor
     fxPivotMotor = new TalonFX(Constants.Claw.ClawPivotMotor.MOTOR_ID); 
     //configure motor
     fxPivotConfig = new TalonFXConfiguration();

    // apply configurations
    fxPivotMotor.getConfigurator().apply(fxPivotConfig);
    fxPivotMotor.setNeutralMode(NeutralModeValue.Brake);

    //angleMotionMagic.MotionMagicAcceleration = Constants.Claw.ClawPivotMotor.ACCELERATION;
    //angleMotionMagic.MotionMagicCruiseVelocity = Constants.Claw.ClawPivotMotor.MAX_SPEED;
       // Apply configurations
       fxPivotMotor.getConfigurator().apply(fxPivotConfig); 

        Slot0Configs slot0 = fxPivotConfig.Slot0;
        slot0.kP = Constants.Claw.ClawPivotMotor.KP;
        slot0.kI = Constants.Claw.ClawPivotMotor.KI;
        slot0.kD = Constants.Claw.ClawPivotMotor.KD;
     
  }

  //Sets the speed for to pivot motor
  public void setPivotSpeed(double speedPercent) {
    fxPivotMotor.set(speedPercent);
}

  public void setPivotAngle(double degrees) {
// converts degrees into encoder values
    double targetEncoderValue = degrees * 0.78;

    if (targetEncoderValue < Constants.Claw.ClawPivotMotor.CANCODER_MIN) //MIN is 0 see constants
        targetEncoderValue = Constants.Claw.ClawPivotMotor.CANCODER_MIN;

    else if (targetEncoderValue > Constants.Claw.ClawPivotMotor.CANCODER_MAX) //MAX is 90 see constants
        targetEncoderValue = Constants.Claw.ClawPivotMotor.CANCODER_MAX;

    setPivotAngle(targetEncoderValue);

    fxPivotMotor.setControl(m_mmReq.withPosition(targetEncoderValue).withSlot(0));
}

public void setZero(){
  fxPivotMotor.setPosition(0);
}

public Angle getPivotPosition() {
  return fxPivotMotor.getRotorPosition().getValue();
}
 // COME BACK TO THIS!!!!!!!!!!!!!!!
 
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //   SmartDashboard.putNumber("Claw Pivot Angle", fxPivotMotor.getRotorPosition().getValue());
  }
}
