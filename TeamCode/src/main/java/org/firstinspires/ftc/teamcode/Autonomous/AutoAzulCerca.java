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

// Importamos tus comandos y subsistemas
import org.firstinspires.ftc.teamcode.Commands.AutoCommands;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;

@Autonomous
public class AutoAzulCerca extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Inicio
        Pose2d beginPose =  new Pose2d(-52, -52, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // --- INICIALIZAMOS TUS SUBSISTEMAS ---
        Shooter myShooter = new Shooter(telemetry, hardwareMap);
        Roller myIntake = new Roller(telemetry, hardwareMap);
        Feeder myFeeder = new Feeder(telemetry, hardwareMap);

        // --- CREAMOS EL CONTROLADOR DE COMANDOS ---
        AutoCommands robot = new AutoCommands(myShooter, myIntake, myFeeder);

        // --- TRAYECTORIAS (Tus coordenadas originales) ---

        Action preLoad = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-38, -32), Math.toRadians(225))
                .build();
        Pose2d shootingPose = new Pose2d(-38, -32, Math.toRadians(225));

        Action stack1 = drive.actionBuilder(shootingPose)
                .splineToLinearHeading(new Pose2d(-8,  -28, Math.toRadians(266)), Math.toRadians(0))
                .build();
        Pose2d fstack1 = new Pose2d(-8, -28, Math.toRadians(266));

        Action take1 = drive.actionBuilder(fstack1)
                .strafeToLinearHeading(new Vector2d(-8, -54), Math.toRadians(266))
                .build();
        Pose2d ftake1 = new Pose2d(-8, -54, Math.toRadians(266));

        Action returnShootingPose = drive.actionBuilder(ftake1)
                .strafeToLinearHeading(new Vector2d(-38.1, -32), Math.toRadians(225))
                .build();
        Pose2d fsecondshoot = new Pose2d(-38.1, -32, Math.toRadians(225));

        Action stack2 = drive.actionBuilder(fsecondshoot)
                .splineToLinearHeading(new Pose2d(16,-28, Math.toRadians(266)), Math.toRadians(0))
                .build();
        Pose2d fstack2 = new Pose2d(16, -28, Math.toRadians(266));

        Action take2 = drive.actionBuilder(fstack2)
                .strafeToLinearHeading(new Vector2d(16, -50), Math.toRadians(266))
                .build();
        Pose2d ftake2 = new Pose2d(16, -50, Math.toRadians(266));

        Action returnFromStack2 = drive.actionBuilder(ftake2)
                .strafeToLinearHeading(new Vector2d(-38.2, -32), Math.toRadians(225))
                .build();


        telemetry.addData("Estado", "Listo para arrancar...");
        telemetry.update();
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

                                    new SleepAction(1.25),
                                    robot.feed(3.75), // Disparar

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

                                    new SleepAction(0.5),
                                    robot.feed(3.75), // Disparo fuerte

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

                                    new SleepAction(0.5),
                                    robot.feed(3.75),

                                    // FIN
                                    robot.stopShooter(),
                                    robot.stopCollect()
                            )
                    )
            );
        }
    }
}