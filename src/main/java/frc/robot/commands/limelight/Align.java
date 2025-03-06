package frc.robot.commands.limelight;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

public class Align extends Command {
    public Align() {
        addRequirements(RobotContainer.drivetrain, RobotContainer.vision);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        RobotContainer.vision.align(RobotContainer.drivetrain);
    }
}
