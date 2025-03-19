package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class HoldAlgaeArm extends Command {
    public HoldAlgaeArm() {
        addRequirements(RobotContainer.algae);
    }
    
    @Override
    public void execute() {
        RobotContainer.algae.zero();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
