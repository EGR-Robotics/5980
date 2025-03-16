package frc.robot.commands.actuator;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class StopServo extends Command {
    @Override
    public void execute() {
        System.out.println("Stopping servo");

        RobotContainer.actuator.stop();
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.actuator.getSpeed() == 0;
    }
}
