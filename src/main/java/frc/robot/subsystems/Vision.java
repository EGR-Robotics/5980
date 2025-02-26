package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VISION;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
    private DigitalInput m_hall_effect;
    private Debouncer m_debouncer;

    private SparkMax m_motor;

    private SparkClosedLoopController m_PIDController;
    private RelativeEncoder m_encoder;

    private MutAngle m_targetRotations = Units.Rotations.mutable(Double.NaN);
    private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
            Double.NaN);
    private MutAngle m_currentRotationsHolder = Units.Rotations.mutable(
            Double.NaN);

    

    private final NetworkTable limelightTable;

    public Vision() {
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
        double angleToTarget = VISION.CAMERA_ANGLE + getVerticalOffset();
        return (VISION.TARGET_HEIGHT - VISION.CAMERA_HEIGHT) / Math.tan(Math.toRadians(angleToTarget));
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

    // "proportional control" is a control algorithm in which the output is
    // proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional
    // to the
    // "tx" value from the Limelight.
    double limelight_aim_proportional() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our
        // proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = getHorizontalOffset() * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are
    // different.
    // if your limelight and target are mounted at the same or similar heights, use
    // "ta" (area) for target ranging rather than "ty"
    double limelight_range_proportional() {
        double kP = .1;
        double targetingForwardSpeed = getVerticalOffset() * kP;

        targetingForwardSpeed *= VISION.MaxSpeed;
        targetingForwardSpeed *= -1.0;

        return targetingForwardSpeed;
    }

    public void align(CommandSwerveDrivetrain drivetrain) {
        final var rot_limelight = limelight_aim_proportional();
        var rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        var xSpeed = forward_limelight;

        // while using Limelight, turn off field-relative driving.
        boolean fieldRelative = false;

        double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second

        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(VISION.MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        drivetrain
            .applyRequest(() -> drive.withVelocityX(-0.3 * VISION.MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-0.3 * VISION.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-0.3 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
    }
}
