package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;

    // Camera and Target Constants
    private static final double CAMERA_ANGLE = 25.0; // Adjust based on mounting angle
    private static final double TARGET_HEIGHT = 2.64; // Target height in meters
    private static final double CAMERA_HEIGHT = 0.90; // Camera height in meters

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Checks if the Limelight has a valid target.
     * 
     * @return true if a target is detected, false otherwise.
     */  
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Gets the horizontal offset (tx) from the crosshair to the target.
     * 
     * @return Horizontal offset in degrees.
     */
    public double getHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    /**
     * Gets the vertical offset (ty) from the crosshair to the target.
     * 
     * @return Vertical offset in degrees.
     */
    public double getVerticalOffset() {
        return limelightTable.getEntry("ty").getDouble(0);
    }

    /**
     * Estimates distance to the target using the camera angle and target height.
     * 
     * @return Estimated distance in meters.
     */
    public double getEstimatedDistance() {
        if (!hasTarget()) {
            return -1.0;
        }
        double angleToTarget = CAMERA_ANGLE + getVerticalOffset();
        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(angleToTarget));
    }

    /**
     * Sets the LED mode of the Limelight.
     * 0 = Pipeline default, 1 = Force off, 2 = Force blink, 3 = Force on
     * 
     * @param mode LED mode value.
     */
    public void setLedMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }

    /**
     * Sets the Limelight pipeline.
     * 
     * @param pipeline Pipeline index (0-9).
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    @Override
    public void periodic() {
        // Update SmartDashboard values for debugging
        SmartDashboard.putBoolean("Target Detected", hasTarget());
        SmartDashboard.putNumber("Horizontal Offset (tx)", getHorizontalOffset());
        SmartDashboard.putNumber("Vertical Offset (ty)", getVerticalOffset());
        SmartDashboard.putNumber("Estimated Distance", getEstimatedDistance());
    }
}
