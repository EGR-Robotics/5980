package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.LIMELIGHT;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    boolean useLimelightPoseEstimate = false;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (useLimelightPoseEstimate) {
            var lastResult = LimelightHelpers.getLatestResults(LIMELIGHT.LIMELIGHT_NAME_1);

            Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

            SmartDashboard.putNumberArray("LIMELIGHT_ESTIMATED_POSE", new double[] { llPose.getX(), llPose.getY() });

            if (lastResult.valid)
                RobotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
        }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null)
            m_autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null)
            m_autonomousCommand.cancel();
    }

    @Override

    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
