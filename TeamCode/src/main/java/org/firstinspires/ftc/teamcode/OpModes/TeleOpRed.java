package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Commands;
import org.firstinspires.ftc.teamcode.Commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;

@TeleOp
public class TeleOpRed extends CommandOpMode {
    Roller rollerSubsystem;
    Vision vision;
    Turret turret;

    @Override
    public void initialize() {
        GamepadEx chassisDriver = new GamepadEx(gamepad1);
        double right_trigger = gamepad1.right_trigger; // Recolectar (Gamepad 1)

        MecanumDrive sampleMecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        MecanumDriveSubsystem driveSystem = new MecanumDriveSubsystem(sampleMecanumDrive, true, true);

        rollerSubsystem = new Roller(telemetry, hardwareMap);
        vision = new Vision(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, vision);
        Shooter Shooter = new Shooter(telemetry, hardwareMap);
        Roller Intake = new Roller(telemetry, hardwareMap);
        Feeder Feeder = new Feeder(telemetry, hardwareMap);
        Commands robot = new Commands(Shooter, Intake, Feeder);

        //Chasis
        driveSystem.setDefaultCommand(new MecanumDriveCommand(driveSystem,
                () -> -chassisDriver.getLeftX() * -1, () -> chassisDriver.getLeftY(), chassisDriver::getRightX));

        //Reset Heading
        chassisDriver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(driveSystem::resetFieldCentricHeading));

        //Roller

        //Agarrar
        chassisDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(() -> robot.collect()))
                .whenReleased(new InstantCommand(() -> robot.stopCollect()));

        //Soltar
        chassisDriver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(() -> rollerSubsystem.setPower(-0.65)))
                .whenReleased(new InstantCommand(() -> rollerSubsystem.setPower(0)));

        //Flywheel

        //Lejos
        chassisDriver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> robot.enableShooter()))
                .whenPressed(new InstantCommand(() -> robot.setFarSpeed()));

        //Media
        chassisDriver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> robot.enableShooter()))
                .whenPressed(new InstantCommand(() -> robot.setMediumSpeed()));

        //Cerca
        chassisDriver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> robot.enableShooter()))
                .whenPressed(new InstantCommand(() -> robot.setNearSpeed()));

        //Detener
        chassisDriver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> robot.stopShooter()));

        //Shooter

        if (right_trigger > 0.25) {
            new RunCommand(() -> {
                double power = turret.calculatePID(24);
                turret.move(power);
            }, turret);

            if (robot.isReadyToShoot) {
                new SequentialCommandGroup(
                        new InstantCommand(Feeder::feed),
                        new WaitCommand(250),
                        new InstantCommand(() -> rollerSubsystem.setPower(1))
                );
            } else {
                new ParallelCommandGroup(
                        new InstantCommand(Feeder::close),
                        new InstantCommand(() -> rollerSubsystem.setPower(0))
                );
            }
        } else
            new SequentialCommandGroup(
                    new InstantCommand(() -> turret.stop(), turret),
                    new InstantCommand(Feeder::close),
                    new InstantCommand(() -> rollerSubsystem.setPower(0))
            );






                schedule(new RunCommand(() -> {
            driveSystem.update();
            telemetry.addData("Heading",(driveSystem.getPoseEstimate().heading.toDouble()));
            telemetry.update();
        }));
    }
}
