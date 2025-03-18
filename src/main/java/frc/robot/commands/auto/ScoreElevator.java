package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.arm.SetArmDistance;
import frc.robot.commands.elevator.SetElevatorDistance;

public class ScoreElevator extends ParallelCommandGroup {
    public ScoreElevator() {
        super(
                new SetElevatorDistance(14.5));
    }
}
