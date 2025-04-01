package frc.robot.commands.sensor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DetectCoral extends Command{
    public DetectCoral(){
        addRequirements(RobotContainer.canRange);
    }
    
    @Override
    public void execute() {
        if (RobotContainer.canRange.isAtTarget()){
            RobotContainer.candle.green();
        }
        else{
            RobotContainer.candle.EGR();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
