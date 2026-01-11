package org.firstinspires.ftc.teamcode.Commands;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

// Importamos tus piezas (subsistemas)
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;

public class AutoCommands {
    // Definimos las partes del robot
    private final Shooter shooter;
    private final Roller intake;
    private final Feeder feeder;

    // --- CONFIGURACIÓN DE VELOCIDADES ---
    public static double VEL_MOVIMIENTO = 750; // Velocidad suave (Preload)
    public static double VEL_CERCA = 1050; // Velocidad suave (Preload)
    public static double VEL_LEJOS = 1150; // Velocidad fuerte (Desde el stack)

    // Variable que recuerda a qué velocidad queremos ir ahorita
    private double currentTargetVel = VEL_CERCA;

    private boolean shooterEnabled = false;

    // El constructor recibe tus subsistemas reales
    public AutoCommands(Shooter shooterSubsystem, Roller intakeSubsystem, Feeder feederSubsystem) {
        this.shooter = shooterSubsystem;
        this.intake = intakeSubsystem;
        this.feeder = feederSubsystem;
    }

    // --- ACCIONES DE DISPARO ---

    // 1. Mantiene la velocidad (Corre siempre de fondo)
    public class MaintainShooterSpeed implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (shooterEnabled) {
                // Le dice al shooter que vaya a la velocidad que tengamos seleccionada
                shooter.shoot(currentTargetVel);
            } else {
                shooter.shoot(0); // Apagar
            }
            // Datos para ver en la computadora
            packet.put("Shooter Target", currentTargetVel);
            packet.put("Shooter Enabled", shooterEnabled);
            return true; // Retorna true para nunca morir (sigue corriendo)
        }
    }

    // 2. Cambiar a modo CERCA
    public class SetSpeedNear implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            currentTargetVel = VEL_CERCA;
            return false; // Termina rápido
        }
    }

    // 3. Cambiar a modo LEJOS
    public class SetSpeedFar implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            currentTargetVel = VEL_LEJOS;
            return false; // Termina rápido
        }
    }

    public class SetSpeedMovement implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            currentTargetVel = VEL_MOVIMIENTO;
            return false; // Termina rápido
        }
    }

    // Prender/Apagar Shooter
    public class EnableShooter implements Action {
        @Override public boolean run(@NonNull TelemetryPacket packet) { shooterEnabled = true; return false; }
    }
    public class DisableShooter implements Action {
        @Override public boolean run(@NonNull TelemetryPacket packet) { shooterEnabled = false; return false; }
    }

    // --- ACCIONES DE ALIMENTACIÓN (FEEDER) ---

    // Secuencia de disparo: Abre, empuja con intake, espera y cierra
    public class FeedShooter implements Action {
        private final ElapsedTime timer = new ElapsedTime();
        private final double timeToFeed;
        private boolean initialized = false;

        public FeedShooter(double timeSeconds) { this.timeToFeed = timeSeconds; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                feeder.feed(); // Abrir servomotor
                intake.setPower(1); // Intake ayuda a empujar
                initialized = true;
            }

            if (timer.seconds() < timeToFeed) {
                return true; // Sigue esperando
            } else {
                feeder.close(); // Cierra
                intake.setPower(0); // Apaga intake
                return false; // Termina
            }
        }
    }

    // Solo abrir puerta (para pre-abrir)
    public class OpenGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            feeder.feed();
            return false;
        }
    }

    // --- ACCIONES DE INTAKE (ROLLER) ---

    public class CollectAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            feeder.close(); // Por seguridad, cerramos puerta
            intake.setPower(1.0); // Chupar pixels
            return false;
        }
    }

    public class StopCollect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(0);
            return false;
        }
    }

    // --- LISTA DE COMANDOS (GETTERS) ---
    // Estos son los que llamas desde tu Auto
    public Action maintainShooter() { return new MaintainShooterSpeed(); }
    public Action enableShooter() { return new EnableShooter(); }
    public Action stopShooter() { return new DisableShooter(); }
    public Action setNearSpeed() { return new SetSpeedNear(); }
    public Action setFarSpeed() { return new SetSpeedFar(); }
    public Action setMoveSpeed() { return new SetSpeedMovement(); }


    public Action collect() { return new CollectAction(); }
    public Action stopCollect() { return new StopCollect(); }

    public Action feed(double seconds) { return new FeedShooter(seconds); }
    public Action openGate() { return new OpenGate(); }
}