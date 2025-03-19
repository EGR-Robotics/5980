package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.SCORING;
import frc.robot.commands.arm.HoldArmDistance;
import frc.robot.commands.elevator.SetElevatorDistance;

public class L4 extends ParallelCommandGroup {
    public L4() {
        super(
                new SetElevatorDistance(SCORING.L4_ELEVATOR_POSITION),
                new SequentialCommandGroup(
                        new WaitCommand(0.5).unless(() -> RobotContainer.elevator.getEncoderPosition() < 30),
                        new HoldArmDistance(SCORING.L4_ARM_POSITION))
        );
    }
}
