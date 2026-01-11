package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Commands;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class AutoAzulCerca extends LinearOpMode {
     @Override
    public void runOpMode() {
         Pose2d beginPose =  new Pose2d(-52, -52, Math.toRadians(225));
         MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

         // TRAYECTORIAS
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


         telemetry.addData("Estado", "Esperando inicio...");
        telemetry.update();
        waitForStart();

         if (opModeIsActive()) {
             Actions.runBlocking(
                     new ParallelAction(
                             Commands.maintainShooter(), // PIDF de fondo

                             new SequentialAction(
                                     // --- PRELOAD ---
                                     new ParallelAction(
                                             preLoad,
                                             actions.enableShooter(),
                                             // Truco: Abrir puerta ANTES de llegar (Super rápido)
                                             // Úsalo solo si las pelotas no se caen al moverse
                                             actions.openGate()
                                     ),
                                     // Como ya abrimos puerta, el feed es casi instantáneo
                                     actions.feed(1.5),

                                     // --- IR AL STACK 1 (TODO PARALELO) ---
                                     // Aquí eliminamos el "static collect". El robot se mueve,
                                     // apaga shooter y prende intake AL MISMO TIEMPO.
                                     new ParallelAction(
                                             stack1,                 // Moverse
                                             actions.collect(),      // Prender Intake
                                             actions.stopShooter()   // Apagar Shooter
                                     ),
                                     take1, // Entrar al stack

                                     // --- REGRESO 1 (PREPARANDO DISPARO) ---
                                     new ParallelAction(
                                             returnShootingPose,
                                             actions.stopCollect(),
                                             actions.enableShooter(),
                                             actions.openGate() // Pre-abrir puerta en el camino de regreso
                                     ),

                                     // DISPARO CICLO 1 (Instantáneo)
                                     actions.feed(1.5),

                                     // --- IR AL STACK 2 (TODO PARALELO) ---
                                     new ParallelAction(
                                             stack2,
                                             actions.collect(),
                                             actions.stopShooter()
                                     ),
                                     take2,

                                     // --- REGRESO 2 ---
                                     new ParallelAction(
                                             returnFromStack2,
                                             actions.stopCollect(),
                                             actions.enableShooter(),
                                             actions.openGate() // Pre-abrir
                                     ),

                                     // DISPARO CICLO 2
                                     actions.feed(1.5), // Un poco más largo para asegurar vaciado

                                     // FIN
                                     actions.stopShooter(),
                                     actions.stopCollect()

    }
}
