package org.firstinspires.ftc.teamcode.Commands.;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Commands {
    private final Hardware robot;

    // PIDF
    public static double TARGET_TPS = 1480;
    public static double F = 0.000425;
    public static double P = 0.007;
    public static double I = 0;
    public static double D = 0;

    //Servo
    public static double BLOCK_POS = 0.43;       // Cerrado
    public static double LEFT_OPEN = 0.0;        // Izquierda Abierto
    public static double RIGHT_OPEN = 0.945;     // Derecha Abierto

    // DELAY entre que sube la patita y se activa el transfer
    private static final double SERVO_PHYSICAL_DELAY = 0.45;

    // Potencias de los motores
    private static final double TRANSFER_POWER = 1;
    private static final double INTAKE_POWER = 1;

    // States
    private boolean shooterEnabled = false;
    private boolean isGateOpen = false; // Esto es para saber si está abierto

    public RobotActions(Hardware hardware) {
        this.robot = hardware;
    }

    // --- PIDF (BACKGROUND) ---
    public class MaintainShooterSpeed implements Action {
        private final ElapsedTime timer = new ElapsedTime();
        private double lastError = 0;
        private double integralSum = 0;
        private boolean isFirstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!shooterEnabled) {
                robot.flyWheel.setPower(0);
                return true;
            }

            double currentTPS = robot.flyWheel.getVelocity();
            double error = TARGET_TPS - currentTPS;
            double timerSeconds = timer.seconds();

            if (isFirstRun) { timerSeconds = 0.0001; isFirstRun = false; }

            double derivative = (error - lastError) / timerSeconds;
            if (Math.abs(error) < 200) integralSum += (error * timerSeconds);
            else integralSum = 0;

            timer.reset();
            lastError = error;

            double feedforward = F * TARGET_TPS;
            double pid = (P * error) + (I * integralSum) + (D * derivative);
            robot.flyWheel.setPower(feedforward + pid);

            packet.put("Shooter TPS", currentTPS);
            return true;
        }
    }

    public class EnableShooter implements Action {
        @Override public boolean run(@NonNull TelemetryPacket packet) { shooterEnabled = true; return false; }
    }
    public class DisableShooter implements Action {
        @Override public boolean run(@NonNull TelemetryPacket packet) { shooterEnabled = false; return false; }
    }

    // Esto se activa cuando me dirijo
    public class OpenGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.left.setPosition(LEFT_OPEN);
            robot.right.setPosition(RIGHT_OPEN);
            isGateOpen = true; // Marcamos que ya está abierta
            return false;
        }
    }

    // Feed Optimizado
    public class FeedShooter implements Action {
        private final ElapsedTime runTimer = new ElapsedTime();
        private final ElapsedTime servoTimer = new ElapsedTime();

        private boolean initialized = false;
        private boolean servosCommandedOpen = false;
        private final double timeToFeed;

        public FeedShooter(double timeSeconds) {this.timeToFeed = timeSeconds;}

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double currentTPS = robot.flyWheel.getVelocity();
            double error = TARGET_TPS - currentTPS;
            boolean rpmReady = Math.abs(error) < TOLERANCE;

            if (!initialized) {
                runTimer.reset();
                // Si OpenGate antes, ya estaba abierto.
                // Si no, asumimos cerradas.
                servosCommandedOpen = isGateOpen;
                if(!isGateOpen) servoTimer.reset(); // Solo vamos a resetear timer si vamos a abrir ahorita
                initialized = true;
            }

            packet.put("DEBUG TPS", currentTPS);

            if (runTimer.seconds() < timeToFeed) {
                if (shooterEnabled && rpmReady) {

                    // ABREN SERVOS (Si no estaban abiertos)
                    if (!servosCommandedOpen) {
                        robot.left.setPosition(LEFT_OPEN);
                        robot.right.setPosition(RIGHT_OPEN);
                        isGateOpen = true;
                        servoTimer.reset();
                        servosCommandedOpen = true;
                    }

                    // Si ya estaba abierta desde antes (isGateOpen era true), el delay es 0.
                    boolean timeCheck = servoTimer.seconds() > SERVO_PHYSICAL_DELAY;

                    if (timeCheck) {
                        robot.transfer.setPower(TRANSFER_POWER);
                        robot.intake.setPower(INTAKE_POWER * 0.5);
                        packet.put(">","FIREEEEEEEEEEEEEEEEEEEEEEEEEEE");
                    } else {
                        // Esperando que se abra físicamente
                        robot.transfer.setPower(0);
                        robot.intake.setPower(0);

                        // Reseteamos el runTimer para que no cuente el tiempo de espera (esto está muy chido, dilo en entrevista)
                        runTimer.reset();
                    }

                } else {
                    closeGateInternal();
                    runTimer.reset();
                }
                return true;
            } else {
                closeGateInternal();
                return false;
            }
        }

        private void closeGateInternal() {
            robot.left.setPosition(BLOCK_POS);
            robot.right.setPosition(BLOCK_POS);
            robot.transfer.setPower(0);
            robot.intake.setPower(0);
            isGateOpen = false; // Cerrada
            servosCommandedOpen = false;
        }
    }

    public class CollectAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.left.setPosition(BLOCK_POS);
            robot.right.setPosition(BLOCK_POS);
            isGateOpen = false; // Aseguramos que está cerrado

            robot.intake.setPower(INTAKE_POWER*0.8);
            robot.transfer.setPower(TRANSFER_POWER*0.8);
            return false;
        }
    }

    public class StopCollect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
            return false;
        }
    }

    public class OuttakeAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.intake.setPower(-INTAKE_POWER);
            robot.transfer.setPower(-TRANSFER_POWER);
            return false;
        }
    }

    // GETTERS
    public Action maintainShooter() { return new MaintainShooterSpeed(); }
    public Action enableShooter() { return new EnableShooter(); }
    public Action stopShooter() { return new DisableShooter(); }
    public Action collect() { return new CollectAction(); }
    public Action stopCollect() { return new StopCollect(); }
    public Action outtake() { return new OuttakeAction(); }
    public Action feed(double seconds) { return new FeedShooter(seconds); }
    public Action openGate() { return new OpenGate(); }
