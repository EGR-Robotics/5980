package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoResetArm extends Command {
    public AutoResetArm() {
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void initialize() {
        RobotContainer.arm.setPosition(0);
    }

    @Override
    public boolean isFinished() {
        return 0.1 > Math.abs(RobotContainer.arm.getPosition());
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.arm.stop();
    }
}
