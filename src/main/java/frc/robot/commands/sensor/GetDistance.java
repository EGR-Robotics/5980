package frc.robot.commands.sensor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class GetDistance extends Command{
    public GetDistance(){
        addRequirements(RobotContainer.canRange);
    }
    
    @Override
    public void execute() {
        RobotContainer.canRange.getDistance();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
