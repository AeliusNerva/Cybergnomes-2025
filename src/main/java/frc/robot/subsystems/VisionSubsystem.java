// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
 // Initialize the Limelight here
  }

    public double getTargetAngle() {
        return NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("tx").getDouble(0.0);
    }

    public double getTargetTY() {
        return NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("ty").getDouble(0.0);
    }

    public double getTargetTX() {
        return NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("tx").getDouble(0.0);
    }

    public double[] get3DPose() {
        return NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("camtran").getDoubleArray(new double[6]);
    }
  
}
