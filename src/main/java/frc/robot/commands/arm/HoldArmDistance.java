package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class HoldArmDistance extends Command {
    private double m_position;

    public HoldArmDistance(double position) {
        m_position = position;
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void initialize() {
        RobotContainer.arm.setPosition(m_position);
        // RobotContainer.arm.setTargetRotations(m_rotations);
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
