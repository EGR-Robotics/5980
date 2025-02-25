package frc.robot.commands.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetElevatorDistance extends Command {
    private Distance m_distance;

    public SetElevatorDistance(Distance distance) {
        m_distance = distance;
        addRequirements(RobotContainer.elevator);
    }

    @Override
    public void initialize() {
        RobotContainer.elevator.setTargetDistance(m_distance);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.elevator.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.elevator.stop();
    }
}
