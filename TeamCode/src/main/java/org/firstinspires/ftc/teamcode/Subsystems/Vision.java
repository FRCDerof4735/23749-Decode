package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {

    private AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    private List<AprilTagDetection> detectedtags = new ArrayList<>();
    private Telemetry telemetry;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // 1. Configurar el procesador de AprilTag
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();

        // 2. Iniciar la cámara
        initVisionPortal(hardwareMap);
    }

    // Método para encender la cámara y el procesador
    private void initVisionPortal(HardwareMap hardwareMap) {
        if (visionPortal != null) return;

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    // --- OPTIMIZACIÓN CLAVE ---
    // Este método corre automáticamente todo el tiempo mientras el OpMode está activo.
    @Override
    public void periodic() {
        // Actualiza la lista de tags detectados constantemente
        detectedtags = aprilTag.getDetections();
    }

    // Obtener la lista completa
    public List<AprilTagDetection> getDetectedtags() {
        return detectedtags;
    }

    // Buscar un ID específico de forma segura (sin crashes)
    public AprilTagDetection getIdSPECIFIC(int id) {
        for (AprilTagDetection detection : detectedtags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null; // Devuelve null si no lo encuentra
    }

    // Obtener el ID del primer tag que vea (seguro)
    public int idTag() {
        if (!detectedtags.isEmpty()) {
            return detectedtags.get(0).id;
        }
        return -1; // Devuelve -1 si no ve nada
    }

    // Mostrar datos en el teléfono (Telemetry)
    public void displayDetection(AprilTagDetection detectedId) {
        if (detectedId == null) return;

        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}