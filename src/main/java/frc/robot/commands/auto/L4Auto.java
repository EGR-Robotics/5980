package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.elevator.SetElevatorDistance;
import frc.robot.RobotContainer;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.SCORING;
import frc.robot.commands.arm.SetArmDistance;

public class L4Auto extends ParallelCommandGroup {
    public L4Auto() {
        super(
                new SetElevatorDistance(SCORING.L4_ELEVATOR_POSITION),
                new SequentialCommandGroup(
                        new WaitCommand(0.5).unless(() -> RobotContainer.elevator.getEncoderPosition() < ELEVATOR.ELEVATOR_SAFE_POS),
                        new SetArmDistance(SCORING.L4_ARM_POSITION))
        );
    }
}
