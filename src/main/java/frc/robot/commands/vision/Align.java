package frc.robot.commands.vision;

import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.LIMELIGHT;

import edu.wpi.first.wpilibj2.command.Command;

public class Align extends Command {
    public Align() {
        addRequirements(RobotContainer.drivetrain, RobotContainer.vision);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(LimelightHelpers.getTX(LIMELIGHT.LIMELIGHT_NAME_1)) < 2.5;
    }

    @Override
    public void execute() {
        RobotContainer.drivetrain.setControl(
                RobotContainer.vision.align());
    }
}
