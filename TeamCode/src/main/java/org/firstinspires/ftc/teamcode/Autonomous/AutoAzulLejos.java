package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.AutoCommands;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;

@Autonomous
public class AutoAzulLejos extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Declarar
        Pose2d beginPose =  new Pose2d(-64.125, -9.75, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Shooter Shooter = new Shooter(telemetry, hardwareMap);
        Roller Intake = new Roller(telemetry, hardwareMap);
        Feeder Feeder = new Feeder(telemetry, hardwareMap);

        AutoCommands robot = new AutoCommands(Shooter, Intake, Feeder);

        // TRAYECTORIAS

        Action preLoad = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-60, -11), Math.toRadians(210))
                .build();
        Pose2d shootingPose = new Pose2d(-60, -11, Math.toRadians(210));

        Action stack1 = drive.actionBuilder(shootingPose)
                .splineToLinearHeading(new Pose2d(-8,  -28, Math.toRadians(270)), Math.toRadians(0))
                .build();
        Pose2d fstack1 = new Pose2d(-8, -28, Math.toRadians(270));

        Action take1 = drive.actionBuilder(fstack1)
                .strafeToLinearHeading(new Vector2d(-8, -54), Math.toRadians(270))
                .build();
        Pose2d ftake1 = new Pose2d(-8, -54, Math.toRadians(270));

        Action returnShootingPose = drive.actionBuilder(ftake1)
                .strafeToLinearHeading(new Vector2d(-38.1, -32), Math.toRadians(225))
                .build();
        Pose2d fsecondshoot = new Pose2d(-38.1, -32, Math.toRadians(225));

        Action stack2 = drive.actionBuilder(fsecondshoot)
                .splineToLinearHeading(new Pose2d(16,-28, Math.toRadians(270)), Math.toRadians(0))
                .build();
        Pose2d fstack2 = new Pose2d(16, -28, Math.toRadians(270));

        Action take2 = drive.actionBuilder(fstack2)
                .strafeToLinearHeading(new Vector2d(16, -50), Math.toRadians(270))
                .build();
        Pose2d ftake2 = new Pose2d(16, -50, Math.toRadians(270));

        Action returnFromStack2 = drive.actionBuilder(ftake2)
                .strafeToLinearHeading(new Vector2d(-38.2, -32), Math.toRadians(225))
                .build();
        Pose2d shoot3 = new Pose2d(-38.2, -32, Math.toRadians(270));

        Action stack3 = drive.actionBuilder(shoot3)
                .strafeToLinearHeading(new Vector2d(25, -28), Math.toRadians(270))
                .build();


        telemetry.addData("Estado", "Listo para arrancar...");
        telemetry.update();
        robot.openGate();
        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            // 1. PROCESO DE FONDO (Mantiene la velocidad del shooter siempre)
                            robot.maintainShooter(),

                            // 2. SECUENCIA PRINCIPAL
                            new SequentialAction(
                                    // --- CICLO 1: PRELOAD (Cerca) ---
                                    new ParallelAction(
                                            preLoad,
                                            robot.setNearSpeed(), // Velocidad baja
                                            robot.enableShooter(),
                                            robot.openGate() // Pre-abrir
                                    ),

                                    new SleepAction(0.15),
                                    robot.feed(2.65), // Disparar

                                    // --- IR A STACK 1 ---
                                    new ParallelAction(
                                            stack1,
                                            robot.collect(),      // Prender Intake
                                            robot.setMoveSpeed()   // Apagar Shooter para ahorrar energía
                                    ),
                                    take1, // Entrar al stack a comer

                                    // --- REGRESO 1 (Lejos) ---
                                    new ParallelAction(
                                            returnShootingPose,
                                            robot.stopCollect(),
                                            robot.setNearSpeed(), // Velocidad baja
                                            robot.openGate()
                                    ),

                                    new SleepAction(0.15),
                                    robot.feed(2.65), // Disparo fuerte

                                    // --- IR A STACK 2 ---
                                    new ParallelAction(
                                            stack2,
                                            robot.collect(),
                                            robot.setMoveSpeed()   // Apagar Shooter para ahorrar energía
                                    ),
                                    take2,

                                    // --- REGRESO 2 (Lejos) ---
                                    new ParallelAction(
                                            returnFromStack2,
                                            robot.stopCollect(),
                                            robot.setNearSpeed(), // Velocidad baja
                                            robot.enableShooter(),
                                            robot.openGate()
                                    ),

                                    new SleepAction(0.15),
                                    robot.feed(2.65),

                                    stack3,

                                    // FIN
                                    robot.stopShooter(),
                                    robot.stopCollect()
                            )
                    )
            );
        }
    }
}