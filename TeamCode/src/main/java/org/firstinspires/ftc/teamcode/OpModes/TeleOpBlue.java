package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;

@TeleOp
public class TeleOpBlue extends CommandOpMode {

    private Roller rollerSubsystem;
    private Vision vision;
    private Shooter shooter;
    private Feeder feeder;
    private MecanumDriveSubsystem driveSystem;
    private GamepadEx chassisDriver;

    @Override
    public void initialize() {
        // 1. Hardware Init
        vision = new Vision(hardwareMap, telemetry);
        shooter = new Shooter(telemetry, hardwareMap);
        feeder = new Feeder(telemetry, hardwareMap);
        rollerSubsystem = new Roller(telemetry, hardwareMap);

        MecanumDrive sampleMecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        driveSystem = new MecanumDriveSubsystem(sampleMecanumDrive, true, true);
        chassisDriver = new GamepadEx(gamepad1);

        // 2. FORCE UPDATE (Crucial Fix)
        // This replaces the old loop() logic. It updates the robot's "brain" (IMU/Pose) constantly.
        schedule(new RunCommand(() -> {
            driveSystem.update();
        }));

        // 3. Drive Command
        // Note: I added inverted LeftY because Gamepads usually read UP as -1.
        // If it drives backward when you push forward, remove the minus sign on getLeftY.
        driveSystem.setDefaultCommand(new MecanumDriveCommand(driveSystem,
                () -> chassisDriver.getLeftX(),
                () -> chassisDriver.getLeftY(),  // Inverted Y usually fixes "forward is backward"
                chassisDriver::getRightX));

        // 4. Buttons
        chassisDriver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(driveSystem::resetFieldCentricHeading));

        chassisDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(() -> rollerSubsystem.setPower(1)))
                .whenPressed(feeder::close)
                .whenReleased(new InstantCommand(() -> rollerSubsystem.setPower(0)));

        chassisDriver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(() -> rollerSubsystem.setPower(-0.65)))
                .whenReleased(new InstantCommand(() -> rollerSubsystem.setPower(0)));

        // Flywheel
        chassisDriver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> { shooter.shoot(1150); shooter.setVelLejos(); }));
        chassisDriver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> { shooter.shoot(1000); shooter.setVelMedia(); }));
        chassisDriver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> { shooter.shoot(775); shooter.setVelCerca(); }));
        chassisDriver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> shooter.shoot(0)));

        // Trigger Logic
        new Trigger(() -> chassisDriver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.25)
                .whileActiveContinuous(new RunCommand(() -> {
                    if (shooter.rpmReady) {
                        feeder.feed();
                        rollerSubsystem.setPower(1);
                    } else {
                        feeder.close();
                        rollerSubsystem.setPower(0);
                    }
                }));

        // 5. Telemetry
        schedule(new RunCommand(() -> {
            telemetry.addData("Heading", driveSystem.getPoseEstimate().heading.toDouble());
            telemetry.update();
        }));
    }
}