package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClawSubsystem implements Subsystem {
    private SparkMax motor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController controller;

    private DigitalInput limitSwitch;

    public ClawSubsystem() {
        limitSwitch = new DigitalInput(1);
        motor = new SparkMax(2, MotorType.kBrushless);
        encoder = motor.getEncoder();

        controller = motor.getClosedLoopController();
        encoder.setPosition(0);
        System.out.println(encoder.getPosition());
    }

    public void goToLevel1() {
        System.out.println("Encoder position: " + encoder.getPosition());
        
        motor.set(1);
        controller.setReference(6, ControlType.kMAXMotionPositionControl);
    }

    public Command move() {
        return run(() -> goToLevel1());
    }   

    // public Command stop() {
    //     return run(() -> motor.set(0));
    // }
}
