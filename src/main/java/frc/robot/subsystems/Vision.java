package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LIMELIGHT;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
    private final SwerveRequest.RobotCentric m_robotCentricRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * Align Y component of robot pose to reef apriltag
     * 
     * @return SwerveRequest to apply to swerve subsystem
     */
    public SwerveRequest alignLR() {
        double kp_aim = 0.015;

        double tx = (LimelightHelpers.getTX(LIMELIGHT.LIMELIGHT_NAME_1));

        double rotationSpeed = -tx * kp_aim;

        return m_robotCentricRequest.withVelocityX(0).withVelocityY(rotationSpeed).withRotationalRate(0);
    }

    /**
     * Align X component of robot pose to reef apriltag
     * 
     * @return SwerveRequest to apply to swerve subsystem
     */
    public SwerveRequest alignTA() {
        double kp_distance = 0.015;

        double tA = (LimelightHelpers.getTA(LIMELIGHT.LIMELIGHT_NAME_1));

        if (tA > LIMELIGHT.TA_TARGET_DISTANCE) {
            tA = -tA;

            kp_distance = 0.0075;
        }

        double distanceSpeed = -tA * kp_distance;

        return m_robotCentricRequest.withVelocityX(distanceSpeed).withVelocityY(0).withRotationalRate(0);
    }

    /**
     * Seed swerve robot pose with megatag 2 pose estimate
     */
    public void megatag2UpdatePose() {
        var driveState = RobotContainer.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        
        LimelightHelpers.SetRobotOrientation(LIMELIGHT.LIMELIGHT_NAME_1, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT.LIMELIGHT_NAME_1);
        
        // Cut out tags only if not reef
        
        // If there are tags and the robot is not currently rotating
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
            // Tune standard deviations
            RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
    }

    /**
     * Align X and Y component of robot pose to reef apriltag (megatag 2)
     * 
     * (for accuracy - requires robot to be near april tag)
     * 
     * @return SwerveRequest to apply to swerve subsystem
     */
    public SwerveRequest alignMegatag2() {
        return m_robotCentricRequest;
    }
}
