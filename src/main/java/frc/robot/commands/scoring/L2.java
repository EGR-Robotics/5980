package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.SCORING;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.HoldArmDistance;
import frc.robot.commands.elevator.SetElevatorDistance;

public class L2 extends ParallelCommandGroup {
    public L2() {
        super(
                new SetElevatorDistance(SCORING.L2_ELEVATOR_POSITION),
                new SequentialCommandGroup(
                        new WaitCommand(0.5).unless(
                                () -> RobotContainer.elevator.getEncoderPosition() < ELEVATOR.ELEVATOR_SAFE_POS),
                        new HoldArmDistance(SCORING.L2_ARM_POSITION)));
    }
}
