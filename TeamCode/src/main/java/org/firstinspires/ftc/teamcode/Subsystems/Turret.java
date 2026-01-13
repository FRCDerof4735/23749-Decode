package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Turret extends SubsystemBase {

    private final DcMotorEx turretMotor;
    private final Vision vision;
    private final PIDController pidController;

    // Variables públicas para ajustar el PID fácil (Tuning)
    public static double kP = 0.0018875;
    public static double kI = 0.0;
    public static double kD = 0.766;

    // El centro de la cámara (1280 / 2 = 640)
    public static double TARGET_CENTER = 640;
    public static double TOLERANCE = 7.5; // Margen de error aceptable

    public Turret(HardwareMap hardwareMap, Vision vision) {
        this.vision = vision;

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDController(kP, kI, kD);
    }

    /**
     * Calcula automáticamente la velocidad para centrarse en el Tag.
     * Retorna 0 si no ve el tag o si ya llegó.
     */
    public double calculatePID(int targetTagId) {
        // Busca el tag usando tu clase Vision optimizada
        AprilTagDetection tag = vision.getIdSPECIFIC(targetTagId);

        // Si no ve el tag, no hace nada
        if (tag == null) {
            return 0;
        }

        double currentX = tag.center.x;

        // Si ya estamos muy cerca del centro (dentro de la tolerancia), paramos
        if (Math.abs(TARGET_CENTER - currentX) < TOLERANCE) {
            return 0;
        }

        // Calcula cuánto mover el motor para llegar a 640
        // Nota: Si gira al revés, cambia el signo a (-)pidController...
        return pidController.calculate(currentX, TARGET_CENTER);
    }

    /**
     * Mueve el motor con una potencia específica.
     */
    public void move(double power) {
        turretMotor.setPower(power);
    }

    /**
     * Detiene el motor por completo.
     */
    public void stop() {
        turretMotor.setPower(0);
    }
}