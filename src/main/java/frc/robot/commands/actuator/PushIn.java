package frc.robot.commands.actuator;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class PushIn extends Command {
    @Override
    public void execute() {
        System.out.println("Pushin command running. Cur speed: " + RobotContainer.actuator.getSpeed() + "...; cur position: " + RobotContainer.actuator.getPosition());


        RobotContainer.actuator.setSpeed(-1);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.actuator.getPosition() == 0;
    }
}
