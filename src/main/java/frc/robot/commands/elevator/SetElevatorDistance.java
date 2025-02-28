package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetElevatorDistance extends Command {
    private Angle m_angle;
    private Distance m_distance;

    public SetElevatorDistance(Distance rot) {
        m_distance = rot;
        addRequirements(RobotContainer.elevator);
    }

    public SetElevatorDistance(Angle rot) {
        m_angle = rot;
        addRequirements(RobotContainer.elevator);
    }

    @Override
    public void initialize() {
        // System.out.println("Running l4 command");
        // RobotContainer.elevator.setTargetRotations(m_angle);
        RobotContainer.elevator.setTargetDistance(m_distance);
    }

    @Override
    public boolean isFinished() {
        // System.out.println("l4 finished function runs");
        // System.out.println(RobotContainer.elevator.getDistance().in(Feet));

        return RobotContainer.elevator.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.elevator.stop();
    }
}
