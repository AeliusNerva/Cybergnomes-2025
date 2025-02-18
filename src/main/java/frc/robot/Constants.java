// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * hello
 */
public final class Constants {
        public static final double STICK_DEAD_BAND = 0.1;
        
   public static final class Swerve {
// CHANGE WITH KRAKEN INFO!!!!
        //public static final int pigeonID = 1;

        // Always ensure Gyro is CCW+ CW-
        public static final boolean INVERT_GYRO = true;

        private static final COTSTalonFXSwerveConstants chosenModule =
                COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        /* Drivetrain Constants */
        // Distance from left wheel to right
        private static final double TRACK_WIDTH = Units.inchesToMeters(25);
        // Distance from front wheel to back
        private static final double WHEEL_BASE = Units.inchesToMeters(22.25);
        public static final double WHEEL_CIRCUMFERENCE = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = chosenModule.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue DRIVE_MOTOR_INVERT = chosenModule.driveMotorInvert;
        public static final InvertedValue ANGLE_MOTOR_INVERT = chosenModule.angleMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_INVERT = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int driveCurrentLimit = 35;
        public static final int DIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = chosenModule.angleKP;
        public static final double ANGLE_KI = chosenModule.angleKI;
        public static final double ANGLE_KD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 1.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32;
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 4.5;

        /* Distance from center of robot to wheels */
        public static final double DRIVE_BASE_RADIUS = 0.43;

        /* Neutral Modes */
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final class Mod0 {
                public static final int DRIVE_MOTOR_ID = 1;
                public static final int ANGLE_MOTOR_ID = 11;
                public static final int CANCODER_ID = 21;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(333.89);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                    DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
            }
    
            /* Front Right Module - Module 1 */
            public static final class Mod1 {
                public static final int DRIVE_MOTOR_ID = 2;
                public static final int ANGLE_MOTOR_ID = 12;
                public static final int CANCODER_ID = 22;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(40.78);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                    DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
                        CANCODER_ID, ANGLE_OFFSET);
            }
    
            /* Back Left Module - Module 2 */
            public static final class Mod2 {
                public static final int DRIVE_MOTOR_ID = 3;
                public static final int ANGLE_MOTOR_ID = 13;
                public static final int CANCODER_ID = 23;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-355.25);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                    DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
            }
    
            /* Back Right Module - Module 3 */
            public static final class Mod3 {
                public static final int DRIVE_MOTOR_ID = 4;
                public static final int ANGLE_MOTOR_ID = 14;
                public static final int CANCODER_ID = 24;
                public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-63.11);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                    DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
            }
   }

   public static final class Elevator {
        //Highest and lowest height that the elevator reaches
        public static final double CANCODER_MIN = 0;
        public static final double CANCODER_MAX = 100;
   }

   public static final class AlgaeIntake {
        //Single falcon500 motor for algae intake
        public static final class AlgaeIntakeMotor {
                public static final int MOTOR_ID = 0; //FILL IN ID ONCE CONFIGURED!
        }
   }

   public static final class ClawPivot {
        //single falcon500 motor for the claw to pivot
        public static final class ClawPivotMotor{
                public static final double MOTOR_ID = 0; //FILL IN ID ONCE CONFIGURED!
        }
   }

   public static final class VisionConstants {
        public static final double REEF_APRILTAG_HEIGHT = 0.6875;
        public static final double PROCCESSOR_APRILTAG_HEIGHT = 0.45875;
        public static final double CORAL_APRILTAG_HEIGHT = 0.5325;
   }

   public static final class Limelight {
        // height and angle of the right limelight
        public static final class Right {
            public static final String NAME = "limelight-gnomesr";

            public static double ANGLE = 0;
            public static double HEIGHT = 0.42008; 
        }
        // height and angle of the left limelight
        public static final class Left {
            public static final String NAME = "limelight-gnomesl";

            public static double ANGLE = 0;
            public static double HEIGHT = 0.42008;
        }

        public static final class Pipelines {
           
        }
    }

}
