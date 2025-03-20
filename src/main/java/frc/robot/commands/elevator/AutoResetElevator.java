package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


public class AutoResetElevator extends Command {
    public AutoResetElevator() {
        addRequirements(RobotContainer.elevator);
    }

    @Override
    public void initialize() {
        RobotContainer.elevator.setEncoderPosition(0);
    }

    @Override
    public boolean isFinished() {
        return 0.1 > Math.abs(RobotContainer.elevator.getEncoderPosition());
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.elevator.stop();
    }
}
