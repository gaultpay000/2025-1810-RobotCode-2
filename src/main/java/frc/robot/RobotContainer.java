// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Intake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

    private final CommandXboxController xbox = new CommandXboxController(0);
    private final CommandJoystick joystick = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem intakeSubsysem = new IntakeSubsystem();


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);


    public RobotContainer() {
        configureXbox();
    }


    @SuppressWarnings("unused")
    private void configureXbox() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((deadzone(-xbox.getLeftY()) * MaxSpeed) * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY((deadzone(-xbox.getLeftX()) * MaxSpeed) * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(deadzone(xbox.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        xbox.b().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))
        // 

        // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        xbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        xbox.x().whileTrue(new Intake(intakeSubsysem, true));
        xbox.y().whileTrue(new Intake(intakeSubsysem, false));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    @SuppressWarnings("unused")
    private void configureJoystick() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((deadzone(-joystick.getY()) * MaxSpeed) * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY((deadzone(-joystick.getX()) * MaxSpeed) * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(deadzone(joystick.getZ()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.button(1).whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.button(12).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    /**
     * Applies a deadzone to the input value. Values within the range of -0.1 to 0.1 are
     * set to zero to prevent small unintentional movements. Values outside this range
     * are squared to maintain the direction while providing finer control at lower speeds.
     *
     * @param input the input value to be adjusted
     * @return the adjusted value after applying the deadzone and squaring
     */
    public double deadzone(double input) {
        //TODO: Work with drivers to find deadzone
        if (Math.abs(input) <= .1 && Math.abs(input) > 0) {
            return 0;
        }

        return input * input; //TODO: Find out if this is necessary
    }
}
