package frc.robot.commands.Algae;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveAlgae extends Command {
    private boolean m_up;

    public MoveAlgae(boolean up) {
        m_up = up;
    }

    @Override
    public void execute() {
        RobotContainer.algae.moveArm(m_up);
    }

    @Override
    public void end(boolean interrupted) {
        // RobotContainer.algae.stop();
    }
}
