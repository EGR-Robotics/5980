package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Brake extends Command {
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public Brake() {
        addRequirements(RobotContainer.drivetrain);
    }

    @Override
    public void execute() {
        RobotContainer.drivetrain.setControl(brake);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
