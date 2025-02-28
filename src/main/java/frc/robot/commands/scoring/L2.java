package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.SCORING;
import frc.robot.commands.arm.SetArmDistance;
import frc.robot.commands.elevator.SetElevatorDistance;

public class L2 extends ParallelCommandGroup {
    public L2() {
        super(
                new SetElevatorDistance(SCORING.L2_ELEVATOR_HEIGHT),
                new SequentialCommandGroup(
                        new WaitCommand(1).unless(() -> RobotContainer.elevator.getDistance()
                                .compareTo(ELEVATOR.ELEVATOR_SAFE_HEIGHT) == -1),
                        new SetArmDistance(SCORING.L2_ARM_ANGLE)));
    }
}
