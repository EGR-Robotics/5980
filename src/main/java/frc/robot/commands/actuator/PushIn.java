package frc.robot.commands.actuator;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class PushIn extends Command {
    @Override
    public void execute() {
        System.out.println("Push in init: " + RobotContainer.actuator.getPosition());

        RobotContainer.actuator.setSpeed(-1);
        // RobotContainer.actuator.setPosition(-1);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Pushin command running. Cur speed: " + RobotContainer.actuator.getSpeed() + "...; cur position: " + RobotContainer.actuator.getPosition());

        if(RobotContainer.actuator.getPosition() <= 0) {
            // RobotContainer.actuator.stop();
            System.out.println("Stopping after pushin command finishes");

            return true;
        }

        return false;
    }
}
