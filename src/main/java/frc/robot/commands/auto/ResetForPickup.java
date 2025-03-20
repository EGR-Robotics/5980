package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmDistance;
import frc.robot.commands.elevator.SetElevatorDistance;

import frc.robot.commands.arm.AutoResetArm;
import frc.robot.commands.elevator.AutoResetElevator;

public class ResetForPickup extends SequentialCommandGroup {
    public ResetForPickup() {
        super(
            new AutoResetArm(),
            new AutoResetElevator()
        );
    }
}
