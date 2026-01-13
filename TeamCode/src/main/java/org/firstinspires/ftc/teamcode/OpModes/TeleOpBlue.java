package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Commands;
import org.firstinspires.ftc.teamcode.Commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;

@TeleOp
public class TeleOpBlue extends CommandOpMode {
    Roller rollerSubsystem;
    Vision vision;
    Turret turret;

    @Override
    public void initialize() {
        GamepadEx chassisDriver = new GamepadEx(gamepad1);

        MecanumDrive sampleMecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        MecanumDriveSubsystem driveSystem = new MecanumDriveSubsystem(sampleMecanumDrive, true, true);

        rollerSubsystem = new Roller(telemetry, hardwareMap);
        vision = new Vision(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, vision);
        Shooter Shooter = new Shooter(telemetry, hardwareMap);
        Roller Intake = new Roller(telemetry, hardwareMap);
        Feeder Feeder = new Feeder(telemetry, hardwareMap);
        Commands robot = new Commands(Shooter, Intake, Feeder);

        // Chasis
        driveSystem.setDefaultCommand(new MecanumDriveCommand(driveSystem,
                () -> -chassisDriver.getLeftX() * -1, chassisDriver::getLeftY, chassisDriver::getRightX));

        // Reset Heading
        chassisDriver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(driveSystem::resetFieldCentricHeading));

        // --- ROLLER (FIXED) ---
        // Uses the NEW direct methods from Commands.java
        chassisDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(robot::startCollect))
                .whenReleased(new InstantCommand(robot::stopIntake));

        chassisDriver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(() -> rollerSubsystem.setPower(-0.65)))
                .whenReleased(new InstantCommand(() -> rollerSubsystem.setPower(0)));

        // --- SHOOTER (FIXED) ---
        // Lejos
        chassisDriver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    robot.setShooterState(true);
                    robot.setSpeedFar();
                }));

        // Media
        chassisDriver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    robot.setShooterState(true);
                    robot.setSpeedMedium();
                }));

        // Cerca
        chassisDriver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    robot.setShooterState(true);
                    robot.setSpeedNear();
                }));

        // Detener
        chassisDriver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> robot.setShooterState(false)));


        // --- MAIN LOOP ---
        schedule(new RunCommand(() -> {
            driveSystem.update();

            // ESSENTIAL: Keep the shooter updating every loop
            robot.updateShooter();

            // Trigger Logic (Must be inside loop to work)
            if (gamepad1.right_trigger > 0.25) {
                double power = turret.calculatePID(20);
                turret.move(power);

                if (robot.isReadyToShoot) {
                    Feeder.feed();
                    rollerSubsystem.setPower(1);
                } else {
                    Feeder.close();
                    rollerSubsystem.setPower(0);
                }
            } else {
                turret.stop();
                Feeder.close();
                rollerSubsystem.setPower(0);
            }

            telemetry.addData("Heading", driveSystem.getPoseEstimate().heading.toDouble());
            telemetry.update();
        }));
    }
}