package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class MecanumDriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private double headingOffSet = 0.0;
    private boolean isInverted = false; // Restored

    public MecanumDriveSubsystem(MecanumDrive drive, boolean isFieldCentric, boolean isBlueAlliance) {
        this.drive = drive;
    }

    // --- RESTORED METHODS FOR AUTO ---

    public MecanumDrive getDrive() {
        return drive;
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.localizer.setPose(pose);
    }

    public void setDrivePower(PoseVelocity2d drivePower) {
        drive.setDrivePowers(drivePower);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public void toggleInverted(){
        this.isInverted = !isInverted;
    }

    // --------------------------------

    public void update() {
        drive.updatePoseEstimate();
    }

    // Restored this alias just in case Auto calls it
    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void resetFieldCentricHeading() {
        // Kept the fix: Use Radians directly
        headingOffSet = drive.localizer.getPose().heading.toDouble();
    }

    public void drive(double leftY, double leftX, double rightX) {
        Pose2d estimate = drive.localizer.getPose();

        // Fix: Ensure rotation math is in Radians
        Vector2d input = rotated(-estimate.heading.toDouble() + headingOffSet, leftX, leftY);

        drive.setDrivePowers(new PoseVelocity2d(
                input,
                -rightX
        ));
    }

    public Vector2d rotated(double angle, double x, double y) {
        double newX = x * cos(angle) - y * sin(angle);
        double newY = x * sin(angle) + y * cos(angle);
        return new Vector2d(newX, newY);
    }

    public Pose2d getPoseEstimate() {
        return drive.localizer.getPose();
    }
}