package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;

// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.AlgaeSubsystem;
// import frc.robot.subsystems.ClawSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Initialize controllers
    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    // private final CommandXboxController controllerJoystick = new CommandXboxController(1);

    // Initialize subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // public final ClawSubsystem claw = new ClawSubsystem();
    // public final AlgaeSubsystem algae = new AlgaeSubsystem();
    // public final VisionSubsystem vision = new VisionSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                          // negative X (left)
                ));

        // Algae bar movement
        // controllerJoystick.a().whileTrue(algae.moveElevatorDownCommand());
        // controllerJoystick.a().onFalse(algae.holdElevatorPositionCommand());

        // Claw Movement Setup
        // controllerJoystick.a().whileTrue(claw.goToLevel1Command());
        // controllerJoystick.a().onFalse(claw.holdCommand());

        // controllerJoystick.leftBumper().whileTrue(claw.moveArmUpCommand());
        // controllerJoystick.leftBumper().onFalse(claw.holdArmPositionCommand());

        // controllerJoystick.rightBumper().whileTrue(claw.moveArmDownCommand());
        // controllerJoystick.rightBumper().onFalse(claw.holdArmPositionCommand());

        // // Elevator movement setup
        // controllerJoystick.leftTrigger().whileTrue(claw.moveElevatorUpCommand());
        // controllerJoystick.leftTrigger().onFalse(claw.holdElevatorPositionCommand());

        // controllerJoystick.rightTrigger().whileTrue(claw.moveElevatorDownCommand());
        // controllerJoystick.rightTrigger().onFalse(claw.holdElevatorPositionCommand());

        // controllerJoystick.b().onTrue(claw.dropCoralCommand());

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.b().whileTrue(
                drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(0,0))));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Example Auto");
    }
}
