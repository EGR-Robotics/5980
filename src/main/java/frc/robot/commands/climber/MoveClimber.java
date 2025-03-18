package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class MoveClimber extends Command {
    double m_velocity;

    public MoveClimber(double velocity) {
        m_velocity = velocity;

        // addRequirements(RobotContainer.climber);
    }

    @Override
    public void execute() {
        // RobotContainer.climber.setClimberSpeed(m_velocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // RobotContainer.climber.setClimberSpeed(m_velocity);
    }
}
