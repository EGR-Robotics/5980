package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetElevatorDistance extends Command {
    private double m_pos;

    public SetElevatorDistance(double pos) {
        m_pos = pos;
        addRequirements(RobotContainer.elevator);
    }

    @Override
    public void initialize() {
        RobotContainer.elevator.setEncoderPosition(m_pos);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Cur elevator distance: " + RobotContainer.elevator.getEncoderPosition());
        return RobotContainer.elevator.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.elevator.stop();
    }
}
