package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.SCORING;
import frc.robot.commands.arm.SetArmDistance;

public class L4LowerCoral extends ParallelCommandGroup {
    public L4LowerCoral() {
        super(
                new SetArmDistance(SCORING.L4_ARM_POSITION + 8));
    }
}
