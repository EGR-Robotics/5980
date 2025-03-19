package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.elevator.SetElevatorDistance;

public class Pickup extends ParallelCommandGroup {
    public Pickup() {
        super(
                new SetElevatorDistance(12));
    }
}
