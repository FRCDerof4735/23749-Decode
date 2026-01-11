package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter extends SubsystemBase {

    DcMotorEx shooterDer;
    DcMotorEx shooterIzq;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    PIDFCoefficients shootPID;


    public Shooter(Telemetry telemetry, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        shootPID = new PIDFCoefficients(24,0.00,0,22.5);

        shooterIzq = hardwareMap.get(DcMotorEx.class, "shooterIzq");
        shooterIzq.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterIzq.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterIzq.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterIzq.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shootPID);


        shooterDer = hardwareMap.get(DcMotorEx.class, "shooterDer");
        shooterIzq.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterDer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterDer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterDer.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterIzq.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shootPID);


    }

    public void shoot(double Vel){
        shooterIzq.setVelocity(-Vel);
        shooterDer.setVelocity(Vel);
    }

    @Override
    public void periodic(){
        telemetry.addData("Izq Vel",shooterIzq.getVelocity());
        telemetry.addData("Der Ve",shooterDer.getVelocity());
    }


}