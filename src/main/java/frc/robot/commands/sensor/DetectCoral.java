package frc.robot.commands.sensor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DetectCoral extends Command{
    private final Timer timer = new Timer();

    public DetectCoral(){
        addRequirements(RobotContainer.canRange);
    }
    
    @Override
    public void execute() {
        if (RobotContainer.canRange.isAtTarget()){
            RobotContainer.candle.green();
            
            timer.start();
        }
        else{
            RobotContainer.candle.GetCoral();

            timer.reset();
            timer.stop();
        }
    }
    
    @Override
    public boolean isFinished() {
        if (timer.get() >= 1){
            timer.stop();
            RobotContainer.candle.EGR();
            
            return true;
        }

        return false;
    }
}
