package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetArmDistance extends Command {
    private double m_position;

    public SetArmDistance(double position) {
        m_position = position;
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void initialize() {
        RobotContainer.arm.setPosition(m_position);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Current arm position: " + RobotContainer.arm.getPosition() + " and is at target? "
                + RobotContainer.arm.isAtTarget());
        return RobotContainer.arm.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.arm.stop();
    }
}
