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

public class L4AutoLower extends ParallelCommandGroup {
    public L4AutoLower() {
        super(
                new SetArmDistance(SCORING.L4_ARM_POSITION + 5)
        // new SetElevatorDistance(-10),
        // new SequentialCommandGroup(
        // new WaitCommand(0.5).unless(() ->
        // RobotContainer.elevator.getEncoderPosition() < -3),
        // )
        );
    }
}
