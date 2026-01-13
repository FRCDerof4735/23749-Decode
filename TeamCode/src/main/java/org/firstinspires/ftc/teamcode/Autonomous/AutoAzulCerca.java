package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Commands;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;

@Autonomous
public class AutoAzulCerca extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Declarar
        Pose2d beginPose = new Pose2d(-52, -52, Math.toRadians(235));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Shooter Shooter = new Shooter(telemetry, hardwareMap);
        Roller Intake = new Roller(telemetry, hardwareMap);
        Feeder Feeder = new Feeder(telemetry, hardwareMap);
        Commands robot = new Commands(Shooter, Intake, Feeder);

        // Bajar Velocidad Movimiento
        VelConstraint velComer = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(15),
                new AngularVelConstraint(Math.toRadians(Math.PI))));

        // Trayectorias
        Action preLoad = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-36, -34), Math.toRadians(230))
                .build();
        Pose2d shootingPose = new Pose2d(-36, -34, Math.toRadians(230));

        Action stack1 = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(new Vector2d(-5.5, -22), Math.toRadians(270))
                .build();
        Pose2d fstack1 = new Pose2d(-5.5, -22, Math.toRadians(270));

        Action take1 = drive.actionBuilder(fstack1)
                .strafeToLinearHeading(new Vector2d(-5.5, -60), Math.toRadians(270), velComer)
                .build();
        Pose2d ftake1 = new Pose2d(-5.5, -60, Math.toRadians(270));

        Action returnShootingPose = drive.actionBuilder(ftake1)
                .strafeToLinearHeading(new Vector2d(-36, -34), Math.toRadians(230))
                .build();
        Pose2d fsecondshoot = new Pose2d(-36, -34, Math.toRadians(230));

        Action stack2 = drive.actionBuilder(fsecondshoot)
                .strafeToLinearHeading(new Vector2d(19, -22), Math.toRadians(270))
                .build();
        Pose2d fstack2 = new Pose2d(18.5, -22, Math.toRadians(270));

        Action take2 = drive.actionBuilder(fstack2)
                .strafeToLinearHeading(new Vector2d(19, -60), Math.toRadians(270), velComer)
                .build();
        Pose2d ftake2 = new Pose2d(18.5, -60, Math.toRadians(270));

        Action returnFromStack2 = drive.actionBuilder(ftake2)
                .strafeToLinearHeading(new Vector2d(-38, -34), Math.toRadians(230))
                .build();
        Pose2d shoot3 = new Pose2d(-38, -34, Math.toRadians(230));

        Action stack3 = drive.actionBuilder(shoot3)
                .strafeToLinearHeading(new Vector2d(43, -18), Math.toRadians(270))
                .build();
        Pose2d ftake3 = new Pose2d(42.5, -18, Math.toRadians(270));

        Action take3 = drive.actionBuilder(ftake3)
                .strafeToLinearHeading(new Vector2d(42.5, -60), Math.toRadians(270), velComer)
                .build();

        //Init
        telemetry.addData("Estado", "Listo para iniciar");
        telemetry.update();
        robot.openGate();
        waitForStart();

        //Play
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            robot.maintainShooter(),

                            new SequentialAction(
                                    // Ciclo 1
                                    new ParallelAction(
                                            preLoad,
                                            robot.setNearSpeed(),
                                            robot.enableShooter(),
                                            robot.openGate()
                                    ),

                                    new SleepAction(0.5),
                                    robot.feed(2.65),

                                    // Recoger Stack 1
                                    new ParallelAction(
                                            stack1,
                                            robot.collect(),
                                            robot.setMoveSpeed()
                                    ),
                                    take1,

                                    // Ciclo 2
                                    new ParallelAction(
                                            returnShootingPose,
                                            robot.stopCollect(),
                                            robot.setNearSpeed(),
                                            robot.openGate()
                                    ),

                                    new SleepAction(0.25),
                                    robot.feed(2.65),

                                    // Recoger Stack 2
                                    new ParallelAction(
                                            stack2,
                                            robot.collect(),
                                            robot.setMoveSpeed()
                                    ),
                                    take2,

                                    // Ciclo 3
                                    new ParallelAction(
                                            returnFromStack2,
                                            robot.stopCollect(),
                                            robot.setNearSpeed(),
                                            robot.enableShooter(),
                                            robot.openGate()
                                    ),

                                    new SleepAction(0.25),
                                    robot.feed(2.65),

                                    // Recoger Stack 3
                                    new ParallelAction(
                                            stack3,
                                            robot.collect(),
                                            robot.stopShooter()
                                    ),
                                    take3,

                                    robot.stopShooter(),
                                    robot.stopCollect()
                            )
                    )
            );
        }
    }
}