package org.firstinspires.ftc.teamcode.autonomous.commands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystem;

public class TrayectoryFollowerCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final TrajectoryActionBuilder trajectory;
    private Action action;

    public TrayectoryFollowerCommand(MecanumDriveSubsystem drive, TrajectoryActionBuilder trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

        this.action = trajectory.build();
    }

    @Override
    public void execute() {
        System.out.println("Corriendo TrayectoryFollowerCommand");
        Actions.runBlocking(action);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted();
    }


}
