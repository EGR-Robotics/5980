package frc.robot.commands.elevator;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SCORING;

public class MoveElevator extends Command {
    private boolean m_direction;

    public MoveElevator(boolean direction) {
        m_direction = direction;
        addRequirements(RobotContainer.elevator);
    }

    @Override
    public void execute() {
        System.out.println("Elevator Position" + RobotContainer.elevator.getEncoderPosition());
        RobotContainer.elevator.setSpeed(m_direction ? SCORING.ELEVATOR_SPEED : -SCORING.ELEVATOR_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.elevator.stop();
        RobotContainer.elevator.postMove();
    }
}
