package frc.robot.commands.elevator;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveElevator extends Command {
    private boolean m_direction;

    public MoveElevator(boolean direction) {
        m_direction = direction;
        addRequirements(RobotContainer.elevator);
    }

    @Override
    public void execute() {
        RobotContainer.elevator.setSpeed(m_direction ? 15 : -15);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.elevator.stop();
    }
}
