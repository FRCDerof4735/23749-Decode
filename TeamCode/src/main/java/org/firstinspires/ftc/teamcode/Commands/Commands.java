package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;

public class Commands {
    private Shooter shooter = null;
    private final Roller intake;
    private final Feeder feeder;

    public static double VEL_MOVIMIENTO = 600;
    public static double VEL_CERCA = 775;
    public static double VEL_MEDIA = 1000;
    public static double VEL_LEJOS = 1150;

    private double currentTargetVel = VEL_CERCA;
    
    public double TARGET_TPS = currentTargetVel;
    public static double TOLERANCE_TPS = 25;


    private boolean shooterEnabled = false;

    public Commands(Shooter shooterSubsystem, Roller intakeSubsystem, Feeder feederSubsystem) {
        this.shooter = shooterSubsystem;
        this.intake = intakeSubsystem;
        this.feeder = feederSubsystem;
    }

    // Comandos Shooter

    // Mantener Velocidad
    public class MaintainShooterSpeed implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (shooterEnabled) {
                shooter.shoot(currentTargetVel);
            } else {
                shooter.shoot(0);
            }
            packet.put("Shooter Target", currentTargetVel);
            packet.put("Shooter Enabled", shooterEnabled);
            return true;
        }
    }

    // Cambio Velocidad Cerca
    public class SetSpeedNear implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            currentTargetVel = VEL_CERCA;
            return false;
        }
    }

    // Cambio Velocidad Media
    public class setSpeedMedium implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            currentTargetVel = VEL_MEDIA;
            return false;
        }
    }

    // Cambio Velocidad Lejos
    public class SetSpeedFar implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            currentTargetVel = VEL_LEJOS;
            return false;
        }
    }

    // Cambio Velocidad de Movimiento
    public class SetSpeedMovement implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            currentTargetVel = VEL_MOVIMIENTO;
            return false;
        }
    }

    // Prender/Apagar Shooter
    public class EnableShooter implements Action {
        @Override public boolean run(@NonNull TelemetryPacket packet) { shooterEnabled = true; return false; }
    }
    public class DisableShooter implements Action {
        @Override public boolean run(@NonNull TelemetryPacket packet) { shooterEnabled = false; return false; }
    }

    // Comandos Feeder

    // Secuencia de disparo
    public class FeedShooter implements Action {
        private final ElapsedTime timer = new ElapsedTime();
        private final double timeToFeed;
        private boolean initialized = false;

        public FeedShooter(double timeSeconds) { this.timeToFeed = timeSeconds; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                feeder.feed();
                intake.setPower(1);
                initialized = true;
            }

            if (timer.seconds() < timeToFeed) {
                return true;
            } else {
                feeder.close();
                intake.setPower(0);
                return false;
            }
        }
    }

    public class OpenFeeder implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            feeder.feed();
            return false;
        }
    }

    // Comandos Roller

    public class CollectAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            feeder.close(); // Por seguridad, cerramos puerta
            intake.setPower(0.75);
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

    public boolean isReadyToShoot = Math.abs(TARGET_TPS - shooter.shooterVel) < TOLERANCE_TPS;

    // Lista de Comandos

    //Shooter
    public Action maintainShooter() { return new MaintainShooterSpeed(); }
    public Action enableShooter() { return new EnableShooter(); }
    public Action stopShooter() { return new DisableShooter(); }
    public Action setNearSpeed() { return new SetSpeedNear(); }
    public Action setMediumSpeed() { return new setSpeedMedium(); }
    public Action setFarSpeed() { return new SetSpeedFar(); }
    public Action setMoveSpeed() { return new SetSpeedMovement(); }

    //Roller
    public Action collect() { return new CollectAction(); }
    public Action stopCollect() { return new StopCollect(); }

    //Feeder
    public Action feed(double seconds) { return new FeedShooter(seconds); }
    public Action openGate() { return new OpenFeeder(); }
}