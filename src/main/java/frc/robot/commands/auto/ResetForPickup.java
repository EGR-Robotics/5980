package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.SetArmDistance;
import frc.robot.commands.elevator.SetElevatorDistance;

public class ResetForPickup extends ParallelCommandGroup {
    public ResetForPickup() {
        super(
            new SetArmDistance(0),
            new SetElevatorDistance(0)
        );
    }
}
