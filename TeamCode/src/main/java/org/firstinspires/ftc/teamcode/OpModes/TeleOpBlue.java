package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//Mecanum
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystem;
//Subsystems
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class TeleOpBlue extends CommandOpMode {
    Roller rollerSubsystem;
    Shooter shooterSubsystem;
    Feeder feederSubsystem;
    Turret turretSubsystem;
    Vision vision;


    @Override
    public void initialize() {
        GamepadEx chassisDriver = new GamepadEx(gamepad1);
        GamepadEx subsystemsDriver = new GamepadEx(gamepad2);
        MecanumDrive sampleMecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        MecanumDriveSubsystem driveSystem = new MecanumDriveSubsystem(sampleMecanumDrive, true, false);
        rollerSubsystem = new Roller(telemetry, hardwareMap);
        vision = new Vision(hardwareMap, telemetry);
        turretSubsystem = new Turret(hardwareMap, vision);
        shooterSubsystem = new Shooter(telemetry, hardwareMap);
        feederSubsystem = new Feeder(telemetry, hardwareMap);

        //Chasis
        driveSystem.setDefaultCommand(new MecanumDriveCommand(driveSystem,
                () -> -chassisDriver.getLeftX()*-1, () -> chassisDriver.getLeftY(), chassisDriver::getRightX));

        //Reset Heading
        chassisDriver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(driveSystem::resetFieldCentricHeading));

        //Roller

        //Agarrar
        chassisDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(()-> rollerSubsystem.setPower(1)))
                .whenReleased(new InstantCommand(()-> rollerSubsystem.setPower(0)));

        //Soltar
        chassisDriver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(()-> rollerSubsystem.setPower(-0.65)))
                .whenReleased(new InstantCommand(()-> rollerSubsystem.setPower(0)));

        //Feed
        chassisDriver.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new InstantCommand( feederSubsystem::close));

        chassisDriver.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(new InstantCommand( feederSubsystem::feed));

        //Shooter

        //Lejos
        chassisDriver.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new InstantCommand(()-> shooterSubsystem.shoot(1150)))
                .whenReleased(new InstantCommand(()->shooterSubsystem.shoot(0)));

        chassisDriver.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(new InstantCommand(()-> shooterSubsystem.shoot(-1400)))
                .whenReleased(new InstantCommand(()->shooterSubsystem.shoot(0)));

        //Cerca
        subsystemsDriver.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new InstantCommand(()-> shooterSubsystem.shoot(1050)))
                .whenReleased(new InstantCommand(()->shooterSubsystem.shoot(0)));


        //Autoacomodo (Dios Plan)



        schedule(new RunCommand(() -> {
            driveSystem.update();
            telemetry.addData("Heading",(driveSystem.getPoseEstimate().heading.toDouble()));
            telemetry.update();
        }));
    }
}
