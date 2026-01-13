package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Feeder extends SubsystemBase {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Servo Tope;

    public Feeder(Telemetry telemetry, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        Tope = hardwareMap.get(Servo.class, "Tope");
    }

    public void feed(){
        Tope.setPosition(0.30);
    }

    public void close(){
        Tope.setPosition(0.675);
    }

    @Override
    public void periodic(){


    }

}