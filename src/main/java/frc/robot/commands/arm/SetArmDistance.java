package frc.robot.commands.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetArmDistance extends Command {
    private Angle m_rotations;

    public SetArmDistance(Angle rotations) {
        m_rotations = rotations;
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void initialize() {
        RobotContainer.arm.setTargetRotations(m_rotations);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.arm.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.arm.stop();
    }
}
