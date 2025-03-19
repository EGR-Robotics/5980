package frc.robot.commands.auto;

import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.LIMELIGHT;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Align extends Command {
    public Align() {
        addRequirements(RobotContainer.drivetrain, RobotContainer.vision);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Estimated TX: " + LimelightHelpers.getTX("limelight"));
        return Math.abs(LimelightHelpers.getTX(LIMELIGHT.LIMELIGHT_NAME_1)) < 2.5;
    }

    @Override
    public void execute() {
        RobotContainer.drivetrain.setControl(
                RobotContainer.vision.align());
    }
}
