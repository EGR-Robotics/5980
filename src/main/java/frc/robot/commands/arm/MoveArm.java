package frc.robot.commands.arm;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.SCORING;

public class MoveArm extends Command {
    private boolean m_direction;

    public MoveArm(boolean direction) {
        m_direction = direction;
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute() {
        RobotContainer.arm.setSpeed(m_direction ? SCORING.ARM_SPEED : -SCORING.ARM_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.arm.stop();
    }
}
