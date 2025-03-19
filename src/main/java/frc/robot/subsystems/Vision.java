package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LIMELIGHT;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
    public Vision() {
    }

    @Override
    public void periodic() {

    }

    public SwerveRequest align() {
        double kp_aim = 0.015;

        double tx = (LimelightHelpers.getTX(LIMELIGHT.LIMELIGHT_NAME_1));

        double rotationSpeed = -tx * kp_aim;

        SwerveRequest.RobotCentric limelightRotate = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        System.out.println(rotationSpeed);

        return limelightRotate.withVelocityX(0).withVelocityY(rotationSpeed).withRotationalRate(0);
    }
}
