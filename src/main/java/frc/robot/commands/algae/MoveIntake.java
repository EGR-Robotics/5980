package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class MoveIntake extends Command {
    double m_velocity;

    public MoveIntake(double velocity) {
        m_velocity = velocity;

        addRequirements(RobotContainer.algae);
    }

    @Override
    public void execute() {
        // RobotContainer.algae.setIntakeSpeed(m_velocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // RobotContainer.algae.setIntakeSpeed(0);
    }
}
