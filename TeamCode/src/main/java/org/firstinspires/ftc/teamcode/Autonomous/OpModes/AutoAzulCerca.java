package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class AutoAzulCerca extends LinearOpMode{
    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(-52, -52, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Hardware robot = new Hardware(this);
        robot.init();
        RobotActions actions = new RobotActions(robot);

        // TRAYECTORIAS
        Action precargadas = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-38, -32), Math.toRadians(225))
                .build();
        Pose2d shootingPose = new Pose2d(-38, -32, Math.toRadians(225));

        Action GoFirst = drive.actionBuilder(shootingPose)
                .splineToLinearHeading(new Pose2d(-8,  -28, Math.toRadians(266)), Math.toRadians(0))
                .build();
        Pose2d FirstLine = new Pose2d(-8, -28, Math.toRadians(266));

        Action TakeFirst = drive.actionBuilder(FirstLine)
                .strafeToLinearHeading(new Vector2d(-8, -54), Math.toRadians(266))
                .build();
        Pose2d EndFirst = new Pose2d(-8, -54, Math.toRadians(266));

        Action returnShootingPose = drive.actionBuilder(EndFirst)
                .strafeToLinearHeading(new Vector2d(-38.1, -32), Math.toRadians(225))
                .build();
        Pose2d Shoot2 = new Pose2d(-38.1, -32, Math.toRadians(225));

        Action GoSecond = drive.actionBuilder(Shoot2)
                .splineToLinearHeading(new Pose2d(16,-28, Math.toRadians(266)), Math.toRadians(0))
                .build();
        Pose2d SecondLine = new Pose2d(16, -28, Math.toRadians(266));

        Action take2 = drive.actionBuilder(SecondLine)
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
            Actions.runBlocking();
        }

    }
}
