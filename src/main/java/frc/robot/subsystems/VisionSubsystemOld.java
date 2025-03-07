package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import pabeles.concurrency.IntOperatorTask.Max;

public class VisionSubsystemOld extends SubsystemBase {
    private final NetworkTable limelightTable;

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // Camera and Target Constants
    private static final double CAMERA_ANGLE = 25.0; // Adjust based on mounting angle
    private static final double TARGET_HEIGHT = 2.64; // Target height in meters
    private static final double CAMERA_HEIGHT = 0.90; // Camera height in meters

    private final NetworkTableEntry m_camPos;

    double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final Matrix<N3, N1> stdDevs = VecBuilder.fill(.7, .7, 9999999);

    public VisionSubsystemOld() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_camPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace");
    
        var driveState = RobotContainer.drivetrain.getState();
    
        double headingDeg = driveState.Pose.getRotation().getDegrees();
    
        LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
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

        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;

        return targetingForwardSpeed;
    }

    public void alignTX() {
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");

        double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second

        final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
        final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
        final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.driverJoystick.getLeftY(), 0.02))
                * MaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.driverJoystick.getLeftX(), 0.02))
                * MaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(RobotContainer.driverJoystick.getRightX(), 0.02))
                * MaxAngularRate;

        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        
        System.out.println("Speed: " + xSpeed + " " + " ySpeed: " + ySpeed + " rot: " + rot);

        final var speedX = xSpeed;
        final var rotF = rot;

        // Send velocities to the Phoenix Swerve drive command
        RobotContainer.drivetrain.applyRequest(
            () -> drive.withVelocityX(-speedX) // Move forward/backward
                .withVelocityY(-ySpeed) // Move left/right
                .withRotationalRate(-rotF) // Rotate
        );
    }

    public Command alignTXCommand() {
        return run(this::alignTX);
    }

    public double[] align(CommandSwerveDrivetrain drivetrain) {
        PIDController xController = new PIDController(1, 0, 0); // Tune Kp, Ki, Kd
        PIDController yController = new PIDController(1, 0, 0);
        PIDController thetaController = new PIDController(1.0, 0, 0);

        var driveState = drivetrain.getState();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);

            Pose2d currentPose = drivetrain.getState().Pose; // Get current position from odometry
            Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
            Pose2d targerPose2d = targetPose.toPose2d();

            // Calculate velocity adjustments using PID controllers
            double xSpeed = xController.calculate(targetPose.getX());
            double ySpeed = yController.calculate(targetPose.getY());
            double thetaSpeed = thetaController.calculate(targerPose2d.getRotation().getRadians());

            double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second

            System.out.println("Current pose: " + currentPose);
            System.out.println("Target pose: " + targetPose);

            // final var speedXF = xSpeed;
            // final var speedYF = ySpeed;
            // final var rotF = thetaSpeed;

            final var speedXF = limelight_aim_proportional();
            
            final var speedYF = limelight_range_proportional();
            final var rotF = 0;

            // Send velocities to the Phoenix Swerve drive command
            // drivetrain.applyRequest(
            //     () -> drive.withVelocityX(-speedXF) // Move forward/backward
            //         .withVelocityY(-speedYF) // Move left/right
            //         .withRotationalRate(-rotF) // Rotate
            // );

            double[] arr = {-speedXF, -speedYF, -rotF};

            return arr;
        }

        return new double[3];
    }

    public Command alignCommand(CommandSwerveDrivetrain drive) {
        return run(() -> align(drive));
    }
}
