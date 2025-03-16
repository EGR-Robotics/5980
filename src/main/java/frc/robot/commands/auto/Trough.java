package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.SetArmDistance;
import frc.robot.commands.elevator.SetElevatorDistance;

public class Trough extends ParallelCommandGroup {
    public Trough() {
        super(
                new SetElevatorDistance(-40),
                new SetArmDistance(0));
    }
}
