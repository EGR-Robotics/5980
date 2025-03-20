package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SCORING;
import frc.robot.commands.elevator.SetElevatorDistance;

import frc.robot.commands.elevator.MoveElevator;

public class Pickup extends SequentialCommandGroup {
    public Pickup() {
        super(
            new SetElevatorDistance(SCORING.ELEVATOR_PICKUP_POSITION),
            new MoveElevator(false).withTimeout(1)
        );
    }
}
