package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClawSubsystem implements Subsystem {
    private SparkMax motor;
    private AbsoluteEncoder encoder;
    private SparkClosedLoopController controller;

    public ClawSubsystem() {
        motor = new SparkMax(2, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();

        controller = motor.getClosedLoopController();
    }

    public void goToLevel1() {
        controller.setReference(0.5, null);
    }

    public Command move() {
        return run(() -> goToLevel1());
    }
}
