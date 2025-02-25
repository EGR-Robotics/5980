package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;

public class StopElevator extends Command {
    @Override
    public void execute() {
        RobotContainer.elevator.stop();
        addRequirements(RobotContainer.elevator);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean wasInterrupted) {
        RobotContainer.elevator.stop();
    }
}
