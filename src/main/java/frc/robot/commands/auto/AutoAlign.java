package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.vision.AlignLR;
import frc.robot.commands.vision.AlignTA;

public class AutoAlign extends SequentialCommandGroup {
    public AutoAlign() {
        super(
            new AlignTA(),
            new AlignLR()
        );
    }
}
