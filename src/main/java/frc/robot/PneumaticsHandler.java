// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class PneumaticsHandler {
    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final Solenoid s_clawBlock = new Solenoid(PneumaticsModuleType.REVPH, 123); //FILL IN!!!


    public PneumaticsHandler() {
        compressor.enableAnalog(Constants.Pneumatics.MIN_PRESSURE, Constants.Pneumatics.MAX_PRESSURE);
    }

    /** Activate/Deactivate claw
     * @param value true: blocked, false: released
     */
    public void setClimberSolenoid(boolean value) {
        s_clawBlock.set(value);
    }

}
