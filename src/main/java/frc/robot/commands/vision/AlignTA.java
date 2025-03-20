package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

import frc.robot.Constants.LIMELIGHT;

public class AlignTA extends Command {
    public AlignTA() {
        addRequirements(RobotContainer.drivetrain, RobotContainer.vision);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(LimelightHelpers.getTA(LIMELIGHT.LIMELIGHT_NAME_1)) > LIMELIGHT.TA_TARGET_DISTANCE;
    }

    @Override
    public void execute() {
        RobotContainer.drivetrain.setControl(
                RobotContainer.vision.alignTA());
    }
}
