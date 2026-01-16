package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class Turret extends SubsystemBase {

    DcMotorEx turretMotor;
    HardwareMap hardwareMap;


    Vision vision;

    private PIDController controllerL;

    private PIDController controllerC;

    double poseShx = 129;
    double poseShy = 135;

    public double tolerance = 7.5;

    public static double pl = 0.0018875, il = 0, dl = 0.766;

    public static double pc = 0, ic = 0, dc = 0;


    public double PosDec = 640;

    public Turret(HardwareMap hardwareMap, Vision vision) {
        this.hardwareMap = hardwareMap;
        this.vision = vision;
        turretMotor = hardwareMap.get(DcMotorImplEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controllerL = new PIDController(pl, il, dl);
        controllerC = new PIDController(pc, ic, dc);
    }

    //Logica
    public double [] turrCen (Vision vision, int TAGID) {
        List<AprilTagDetection> result = vision.getDetectedtags();
        for (AprilTagDetection tag : result) {
            if (tag.id == TAGID) {
                return new double[]{tag.center.x};
            }
        }
        return null;
    }

    public double errorPID (int TAGID) {
        return PosDec - turrCen(vision, TAGID)[0];
    }

    //Auto stop
    public double TurrPidL (Vision vision, int TAGID) {
        double [] data = turrCen(vision, TAGID);
        if (data == null) {
            return 0;
        }
        if (Math.abs(errorPID(TAGID))<tolerance) {
            return 0;
        } else {
            return controllerL.calculate(PosDec, data[0]);
        }
    }

    public double TurrPidC (Vision vision, int TAGID){
        double [] data = turrCen(vision, TAGID);
        if (data == null) {
            return 0;
        }
        if (Math.abs(errorPID(TAGID))<tolerance) {
            return 0;
        } else {
            return controllerC.calculate(PosDec, data[0]);
        }
    }

    //Aplicar PID
    public void Turretmove (double power) {
        turretMotor.setPower(power);
    }

    public boolean finish (int TAGID) {return Math.abs(errorPID(TAGID)) < tolerance;}




}