package frc.robot.commands;

 /*import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

//import frc.robot.generated.TunerConstants;

public class AlignCommand extends Command {
    private final VisionSubsystem m_Vision; // Uses the vision subsystem to get information from the limelight
    private final Swerve m_Swerve; // Uses the swerve subsystem to move the robot
    private final SwerveRequest.RobotCentric m_alignRequest;

    private final double MaxSpeed;
    private final double MaxAngularRate;

    private final double targetDistance; // Desired distance from the tag 
    private final double targetAngle; // Desired angle relative to the tag

    // Constants to adjust robot based off how far off the robot is from the tag:

    private static final double kP_aim = 0.005; // Proportional gain for aiming
    private static final double kP_range = -0.1; // Proportional gain for ranging
    private static final double kP_horizontal = 0.05; // Reduced proportional gain for horizontal movement
    private static final double distanceTolerance = 0.1; // Tolerance for distance in meters
    private static final double angleTolerance = 1.0; // Degrees tolerance for alignment

    private double lastValidTargetTX = 0.0;
    private double lastValidTargetTY = 0.0;

    private double lastValidTargetAngle = 0.0;
    private double lastHorizontalAdjust = 0.0; // Last horizontal adjustment for smoothing
    
    // Timer to track how long the april tag is out of camera view from the robot
    private final Timer lostDetectionTimer = new Timer();      
    private static final double lostDetectionTimeout = 0.5; // 0.5 seconds timeout for lost detection

    
    public AlignCommand(VisionSubsystem vision, Swerve swerve, double targetDistance, double targetAngle) {
        m_Vision = vision;
        m_Swerve = swerve;
        this.targetDistance = targetDistance;
        this.targetAngle = targetAngle;
        m_alignRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5;
        MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.5;
        addRequirements(m_Vision, m_Swerve);
    }

    // lostDetectionTimer initialized
    @Override
    public void initialize() {
        System.out.println("AlignCommand initialized");
        lostDetectionTimer.reset();
        lostDetectionTimer.start();
    }

    // How the robot will react if it either detects or loses sight of April Tag
    @Override
    public void execute() {
        // offsets (horizontal, vertical, and angle) of the robot relative to the april tag
        double currentTargetTX = m_Vision.getTargetTX();
        double currentTargetTY = m_Vision.getTargetTY();
        double currentTargetAngle = m_Vision.getTargetAngle();

        //  TAG IN SIGHT: Detects if theres a tag. If the values do not equal 0, an April tag is present, and sets all variables (TX, TY, TargetAngle) to the april tags values.
        if (currentTargetTX != 0.0 || currentTargetTY != 0.0 || currentTargetAngle != 0.0) {
            lastValidTargetTX = currentTargetTX;
            lastValidTargetTY = currentTargetTY;
            lastValidTargetAngle = currentTargetAngle;
            lostDetectionTimer.reset();

        // TAG LOST FOR A MOMENT: If no tag is detected (all values = 0) and the robot hasn't exceeded lostDetectionTimeout period, the robot will use the values from the last tag (the code above)
        } else if (lostDetectionTimer.get() < lostDetectionTimeout) {
            currentTargetTX = lastValidTargetTX;
            currentTargetTY = lastValidTargetTY;
            currentTargetAngle = lastValidTargetAngle;

        // TAG COMPLETELY LOST: The tag has been out of sight for too long, exceeding the lostDetectionTimeout period. The variables (TX, TY, angle) will be set to 0.
        } else {
            currentTargetTX = 0.0;
            currentTargetTY = 0.0;
            currentTargetAngle = 0.0;
        }

        // Calculate to adjust closer and align to april tag
        double distanceError = targetDistance - currentTargetTY; //Adjust distance from april tag (TY is fwd/bkwd)
        double horizontalError = -currentTargetTX; // Invert TX for horizontal adjustment. Left is + and right is -, so it has to be inverted so now -TX is left and +TX is right. (TX is how far the target is left/right)
        double distanceAdjust = limelight_range_proportional(distanceError); // calls limelight_range_proportional method to correct fwd/bkwd distance
        double horizontalAdjust = horizontalError * kP_horizontal; // corrects left/right distance
        
        // Smoothing the horizontal adjustment to prevent jerky movement
        horizontalAdjust = (horizontalAdjust + lastHorizontalAdjust) / 2;
        lastHorizontalAdjust = horizontalAdjust;

        // Slow down the robot as it approaches the target
        // double speedFactor = Math.min(1.0, 2/Math.abs(distanceError / targetDistance));
        // distanceAdjust *= speedFactor;
        // horizontalAdjust *= speedFactor;

        // Adjusts rotation to face april tag, by calling limelight_range_proportional method to calculate proper angle.
        double angleError = currentTargetAngle - targetAngle;
        double steeringAdjust = limelight_aim_proportional(angleError);

        System.out.println("AlignCommand executing");
        System.out.println("Distance Adjust: " + distanceAdjust);
        System.out.println("Horizontal Adjust: " + horizontalAdjust);
        System.out.println("Steering Adjust: " + steeringAdjust);

        // sends movement values to swerves (sets m_Swerve.setControl method to values to move them to according position)
        m_Swerve.setControl(
            m_alignRequest
                .withVelocityX(distanceAdjust)  // Forward/backward movement
                .withVelocityY(horizontalAdjust) // Horizontal (lateral) movement
                .withRotationalRate(steeringAdjust) // Rotational correction
        );

        System.out.println("Control Set:");
        System.out.println("VelocityX: " + distanceAdjust);
        System.out.println("VelocityY: " + horizontalAdjust);
        System.out.println("RotationalRate: " + steeringAdjust);
    }

    // Determines if alignment command is complete. If every value are within its tolerances, then it will return true (complete!). 
    @Override
    public boolean isFinished() {
        return Math.abs(lastValidTargetTY - targetDistance) < distanceTolerance && Math.abs(lastValidTargetAngle - targetAngle) < angleTolerance;
    }

    // Executed when command is completed or interrupted
    @Override
    public void end(boolean interrupted) {
        System.out.println("AlignCommand ended");
        m_Swerve.setControl(
            m_alignRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        lostDetectionTimer.stop();
    }

    // Simple proportional turning control with Limelight
    double limelight_aim_proportional(double angleError) {
        double targetingAngularVelocity = angleError * kP_aim;
        System.out.println("Calculated targetingAngularVelocity: " + targetingAngularVelocity);
        targetingAngularVelocity *= MaxAngularRate;
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    // Simple proportional ranging control with Limelight's "ty" value
    double limelight_range_proportional(double distanceError) {
        double targetingForwardSpeed = distanceError * kP_range;
        System.out.println("Calculated targetingForwardSpeed: " + targetingForwardSpeed);
        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }
} */