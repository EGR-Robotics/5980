package frc.robot.commands.sensor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WaitUntilSensed extends Command {
    public WaitUntilSensed() {
        addRequirements(RobotContainer.canRange);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.canRange.isAtTarget();
    }
}
