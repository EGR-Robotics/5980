package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class MoveIntake extends Command {
    boolean m_up;

    public MoveIntake(boolean up) {
        m_up = up;

        addRequirements(RobotContainer.algae);
    }

    @Override
    public void execute() {
        if(m_up) RobotContainer.algae.intake();
        else RobotContainer.algae.outake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.algae.setIntakeSpeed(0);
    }
}
