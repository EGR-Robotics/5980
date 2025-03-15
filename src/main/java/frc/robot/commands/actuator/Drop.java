package frc.robot.commands.actuator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Drop extends SequentialCommandGroup {
    public Drop() {
        super(
                new PushOut(),
                // new WaitCommand(1),
                new StopServo(),
                new PushIn());
    }
}
