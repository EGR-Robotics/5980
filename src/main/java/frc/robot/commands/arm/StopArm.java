package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;

public class StopArm extends Command {
    @Override
    public void execute() {
        RobotContainer.arm.stop();
        addRequirements(RobotContainer.arm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean wasInterrupted) {
        RobotContainer.arm.stop();
    }
}
