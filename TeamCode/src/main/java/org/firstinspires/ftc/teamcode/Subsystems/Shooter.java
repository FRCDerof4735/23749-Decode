package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter extends SubsystemBase {

    DcMotorEx shooterDer;
    DcMotorEx shooterIzq;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    PIDFCoefficients shootPID;

    double VelDer;
    double VelIzq;

    public static double VEL_CERCA = 775;
    public static double VEL_MEDIA = 1000;
    public static double VEL_LEJOS = 1150;

    public double currentTargetVel = VEL_CERCA;

    public double TARGET_TPS = currentTargetVel;
    public static double TOLERANCE_TPS = 30;

    boolean shooterEnabled = false;


    public Shooter(Telemetry telemetry, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        shootPID = new PIDFCoefficients(22,0.00,0,21);

        shooterIzq = hardwareMap.get(DcMotorEx.class, "shooterIzq");
        shooterIzq.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shootPID);
        shooterIzq.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterIzq.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterDer = hardwareMap.get(DcMotorEx.class, "shooterDer");
        shooterDer.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shootPID);
        shooterDer.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterDer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }

    public void shoot(double Vel){
        shooterIzq.setVelocity(Vel);
        shooterDer.setVelocity(Vel);
    }

    public void setVelLejos(){
        currentTargetVel = VEL_LEJOS;
    }
    public void setVelMedia(){
        currentTargetVel = VEL_MEDIA;
    }
    public void setVelCerca() {
        currentTargetVel = VEL_CERCA;
    }

    public double VelProm;
    double error;
    public boolean rpmReady;

    @Override
    public void periodic(){

        VelProm = ((VelDer + VelIzq) / 2);
        error = TARGET_TPS - VelProm;
        rpmReady = Math.abs(error) < TOLERANCE_TPS;

        VelIzq = shooterIzq.getVelocity();
        VelDer = shooterDer.getVelocity();
        telemetry.addData("FlyWheel Right", VelDer);
        telemetry.addData("FlyWheel RPMs", VelProm);
        telemetry.addData("FlyWheel Target", currentTargetVel);
    }


}