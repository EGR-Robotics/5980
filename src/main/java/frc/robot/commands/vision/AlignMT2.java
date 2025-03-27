package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AlignMT2 extends Command {
    public AlignMT2() {
        addRequirements(RobotContainer.drivetrain, RobotContainer.vision);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        RobotContainer.drivetrain.setControl(
                RobotContainer.vision.alignMegatag2());
    }
}
