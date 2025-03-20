package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.SetElevatorDistance;

import frc.robot.commands.elevator.MoveElevator;

public class Pickup extends SequentialCommandGroup {
    public Pickup() {
        super(
            new SetElevatorDistance(22),
            new MoveElevator(false).withTimeout(1)
        );
        // addRequirements(RobotContainer.elevator);
    }

    // @Override
    // public void execute() {
    //     RobotContainer.elevator.setEncoderPosition(30);
    // } 

    // @Override
    // public boolean isFinished() {
    //     return RobotContainer.elevator.isAtTarget();
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     RobotContainer.elevator.stop();
    // }
}
