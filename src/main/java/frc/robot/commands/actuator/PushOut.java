package frc.robot.commands.actuator;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class PushOut extends Command {
    public PushOut() {

    }

    @Override
    public void execute() {
        System.out.println("Push out pos: " + RobotContainer.actuator.getPosition());
        RobotContainer.actuator.setPosition(1);

        RobotContainer.actuator.setSpeed(1);
    }

    @Override
    public boolean isFinished() {
        // System.out.println("Pushout command running. Cur speed: " +
        // RobotContainer.actuator.getSpeed() + "...; cur position: " +
        // RobotContainer.actuator.getPosition());

        // if(RobotContainer.actuator.getPosition() >= 1) {
        // // RobotContainer.actuator.stop();
        // System.out.println("Stopping after pushout command finishes");

        // return true;
        // }

        return false;
    }
}
