package frc.robot.commands.actuator;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class PushOut extends Command {
    @Override
    public void execute() {
        System.out.println("Pushout command running. Cur speed: " + RobotContainer.actuator.getSpeed() + "...; cur position: " + RobotContainer.actuator.getPosition());


        RobotContainer.actuator.setSpeed(1);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.actuator.getPosition() == 1;
    }
}
