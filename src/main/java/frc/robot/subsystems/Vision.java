package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LIMELIGHT;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    public Vision() {
    }

    @Override
    public void periodic() {

    }

    public SwerveRequest alignLR() {
        double kp_aim = 0.015;

        double tx = (LimelightHelpers.getTX(LIMELIGHT.LIMELIGHT_NAME_1));

        double rotationSpeed = -tx * kp_aim;

        SwerveRequest.RobotCentric limelightRotate = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return limelightRotate.withVelocityX(0).withVelocityY(rotationSpeed).withRotationalRate(0);
    }

    public SwerveRequest alignTA() {
        double kp_distance = 0.015;

        double tA = (LimelightHelpers.getTA(LIMELIGHT.LIMELIGHT_NAME_1));

        double distanceSpeed = tA * kp_distance;

        SwerveRequest.RobotCentric limelightRotate = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return limelightRotate.withVelocityX(distanceSpeed).withVelocityY(0).withRotationalRate(0);
    }
}
