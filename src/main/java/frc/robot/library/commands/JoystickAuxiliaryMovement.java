package frc.robot.library.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.library.GenericMotorSubsystem;

/**
 * Generic command to move auxiliary SparkMax motor using joystick
 * 
 * @param subsystem Subsystem for auxiliary component
 * @param controller
 * @param deadband Minimum left or y
 */
public class JoystickAuxiliaryMovement extends Command {
    private GenericMotorSubsystem m_subsystem;
    private XboxController m_controller;
    private double m_deadband;
    private double m_max_speed;

    private boolean m_vertical = true;

    public JoystickAuxiliaryMovement(GenericMotorSubsystem subsystem, XboxController controller, double deadband, double maxSpeed, boolean vertical) {
        m_subsystem = subsystem;
        m_controller = controller;
        m_max_speed = maxSpeed;

        m_vertical = vertical;

        addRequirements(m_subsystem);
    }

    /**
     * Handle vertical movement with Y component of controller
     */
    private void verticalMovementHandler() {
        double leftY = m_controller.getLeftY();

        if (leftY > m_deadband)
            m_subsystem.m_motor.setSpeed(m_max_speed * leftY);
        else if (leftY < -m_deadband)
            m_subsystem.m_motor.setSpeed(-m_max_speed * leftY);
        else
            m_subsystem.m_motor.stop();
    }

    /**
     * Handle horizontal movement with X component of controller
     */
    private void horizontalMovementHandler() {
        double leftX = m_controller.getLeftX();

        if (leftX > m_deadband)
            m_subsystem.m_motor.setSpeed(m_max_speed * leftX);
        else if (leftX < -m_deadband)
            m_subsystem.m_motor.setSpeed(-m_max_speed * leftX);
        else
            m_subsystem.m_motor.stop();
    }

    @Override
    public void execute() {
        if(m_vertical) verticalMovementHandler();
        else horizontalMovementHandler();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
