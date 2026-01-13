package org.firstinspires.ftc.teamcode.Commands;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;

public class Commands {
    private final Shooter shooter;
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
    double VelFlyWheel;

    public Commands(Shooter shooterSubsystem, Roller intakeSubsystem, Feeder feederSubsystem) {
        this.shooter = shooterSubsystem;
        this.intake = intakeSubsystem;
        this.feeder = feederSubsystem;
        VelFlyWheel = shooterSubsystem.getVelocidad();
    }

    // --- ROADRUNNER ACTIONS (Keep these for AUTO) ---
    public class MaintainShooterSpeed implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // This is only for Auto! TeleOp uses updateShooter() below.
            if (shooterEnabled) {
                shooter.shoot(currentTargetVel);
            } else {
                shooter.shoot(0);
            }
            return true;
        }
    }

    public class CollectAction implements Action {
        @Override public boolean run(@NonNull TelemetryPacket packet) {
            feeder.close();
            intake.setPower(0.75);
            return false;
        }
    }

    public class StopCollect implements Action {
        @Override public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(0);
            return false;
        }
    }

    // (Helper actions for auto...)
    public Action maintainShooter() { return new MaintainShooterSpeed(); }
    public Action collect() { return new CollectAction(); }
    public Action stopCollect() { return new StopCollect(); }


    // =========================================================
    // --- DIRECT METHODS FOR TELEOP (USE THESE!) ---
    // =========================================================

    // INTAKE
    public void startCollect() {
        feeder.close();
        intake.setPower(0.75);
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    // SHOOTER SPEED SETTERS
    public void setSpeedNear() { currentTargetVel = VEL_CERCA; }
    public void setSpeedMedium() { currentTargetVel = VEL_MEDIA; }
    public void setSpeedFar() { currentTargetVel = VEL_LEJOS; }

    // SHOOTER STATE
    public void setShooterState(boolean enabled) {
        shooterEnabled = enabled;
    }

    // RUN THIS IN YOUR LOOP
    public void updateShooter() {
        if (shooterEnabled) {
            shooter.shoot(currentTargetVel);
        } else {
            shooter.shoot(0);
        }
        // Update the variable needed for "ReadyToShoot" check
        VelFlyWheel = shooter.getVelocidad();
        TARGET_TPS = currentTargetVel;
        isReadyToShoot = Math.abs(TARGET_TPS - VelFlyWheel) < TOLERANCE_TPS;
    }

    // Simple boolean for checking status
    public boolean isReadyToShoot = false;
}