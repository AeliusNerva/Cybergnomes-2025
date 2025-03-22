// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class PneumaticsHandler {

    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final Solenoid s_coralPusherBlock = new Solenoid(PneumaticsModuleType.REVPH, 0); 
    private final Solenoid s_ClawBlock = new Solenoid(PneumaticsModuleType.REVPH, 4); 
    private final Solenoid s_AlgaeIntake1Block = new Solenoid(PneumaticsModuleType.REVPH, 1);
    private final Solenoid s_AlgaeIntake2Block = new Solenoid(PneumaticsModuleType.REVPH, 2);


    public PneumaticsHandler() {
        compressor.enableAnalog(Constants.Pneumatics.MIN_PRESSURE, Constants.Pneumatics.MAX_PRESSURE);
   }
    /** Activate/Deactivate claw
     * @param value true: blocked, false: released
     */

    public void setCoralPusherSolenoid(boolean value) {
        s_coralPusherBlock.set(value);
    }

    public void setClawSolenoid(boolean value) {
        s_ClawBlock.set(value);
    }

    public void setAlgaeIntake1Solenoid(boolean value) {
        s_AlgaeIntake1Block.set(value);
    }

    public void setAlgaeIntake2Solenoid(boolean value) {
        s_AlgaeIntake2Block.set(value);
    }

    } 
