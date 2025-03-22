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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Claw.PivotMotor;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Pivot;


public class Pivot extends SubsystemBase {
  /** Creates a new ClawPivot. */

  private TalonFX fxPivotMotor;
  private TalonFXConfiguration fxPivotConfig;
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  private final Pivot s_Pivot = RobotContainer.s_Pivot;
  private static Pivot instance = null;

  public double pivotTargetPosition = 0;
  public double pivotPrevPosition = 0;

  private final PIDController pivotController;

  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }
    return instance;
  }

  private double targetAngle;// Target Angle for the claw to pivot in degrees

  public Pivot() {
    // Define motor
     fxPivotMotor = new TalonFX(7); 
     
     //configure motor
     fxPivotConfig = new TalonFXConfiguration();

    // apply configurations
    fxPivotMotor.getConfigurator().apply(fxPivotConfig);
    fxPivotMotor.setNeutralMode(NeutralModeValue.Brake);

    pivotController = new PIDController(0.01, 0, 0 );

    MotionMagicConfigs angleMotionMagic = fxPivotConfig.MotionMagic;
    angleMotionMagic.MotionMagicAcceleration = Constants.Claw.PivotMotor.ACCELERATION;
    angleMotionMagic.MotionMagicCruiseVelocity = Constants.Claw.PivotMotor.MAX_SPEED;

        Slot0Configs slot0 = fxPivotConfig.Slot0;
        slot0.kP = Constants.Claw.PivotMotor.KP;
        slot0.kI = Constants.Claw.PivotMotor.KI;
        slot0.kD = Constants.Claw.PivotMotor.KD;
  }


  //Sets the speed for to pivot motor
  public void setSpeed(double speedPercent) {
    fxPivotMotor.set(speedPercent);
  }

  public double getPosition() {
    return fxPivotMotor.getPosition().getValueAsDouble();
}

  public void setPosition(double position) {
  fxPivotMotor.set(pivotController.calculate(getPosition(), position));
}

public void setTargetPosition( double targetPosition) {
  pivotTargetPosition = targetPosition;
 }

 public void resetPrevPosition( double prevPosition) {
  pivotPrevPosition = prevPosition;
 }

 public boolean isAtTargetPosition() {
  if ( Math.abs(pivotTargetPosition - s_Pivot.getPosition()) < Constants.Claw.PIVOT_POS)
    return true;
  else
    return false;
 }

 public void stopMotor() {
  s_Pivot.stopMotor();
 }
 
 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw pivot position", fxPivotMotor.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
    //   SmartDashboard.putNumber("Claw Pivot Angle", fxPivotMotor.getRotorPosition().getValue());
  }
}
