package frc.robot.commands.actuator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class Drop extends SequentialCommandGroup {
    public Drop() {
        super(
            new PushOut(),
            new PushIn(),
            RobotContainer.actuator.stopCommand()
        );
    }
}
