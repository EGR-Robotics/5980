// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.AbsoluteEncoder;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // private final RobotContainer m_robotContainer;
  private DigitalInput limitSwitch;

  private final CommandXboxController joystick = new CommandXboxController(0);
  private SparkMax motor;

  public Robot() {
    // m_robotContainer = new RobotContainer();
    limitSwitch = new DigitalInput(1);

    motor = new SparkMax(2, MotorType.kBrushless);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override

  public void teleopPeriodic() {
    // System.out.println(m_robotContainer.drivetrain.getPose());

    checkLimitSwitch();
  } 

  public void checkLimitSwitch() {
    if (limitSwitch.get()) {
      motor.set(0);
    } else {
      motor.set(-1);
    }
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
