package frc.robot.commands.Algae;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveAlgae extends Command {
    private boolean m_direction;

    public MoveAlgae(boolean direction) {
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
