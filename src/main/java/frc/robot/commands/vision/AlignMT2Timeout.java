package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class AlignMT2Timeout extends ParallelRaceGroup {
    public AlignMT2Timeout() {
        super(
            new AlignMT2().withTimeout(1.5)
        );
    }
}
