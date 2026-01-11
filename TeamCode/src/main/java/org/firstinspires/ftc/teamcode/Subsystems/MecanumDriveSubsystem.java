package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.cos;

import kotlin.math.*;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

public class MecanumDriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private  boolean fieldCentric;
    private boolean isInverted;
    private final boolean isBlueAlliance;
    private double headingOffSet = 0.0;

    public MecanumDriveSubsystem(MecanumDrive drive, boolean isFieldCentric, boolean isBlueAlliance) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
        this.isBlueAlliance = isBlueAlliance;
    }

    public void toggleInverted(){
        this.isInverted = !isInverted;
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.localizer.setPose(pose);
    }

    public MecanumDrive getDrive() {
        return drive;
    }

    public void update() {
        drive.updatePoseEstimate();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void resetFieldCentricHeading() {
        Pose2d poseEstimate = getPoseEstimate();
        headingOffSet =  Math.toDegrees(drive.localizer.getPose().heading.toDouble()); // Save the current heading as the new "zero"
    }

    public void drive(double leftY, double leftX, double rightX) {
        Pose2d estimate = drive.localizer.getPose();
        Vector2d input;
        input = rotated(-estimate.heading.toDouble() + Math.PI + headingOffSet, leftX, leftY);

        drive.setDrivePowers(new PoseVelocity2d(
                //Joystick Izq
                input,

                //Joystick Der
                -rightX));
    }

    public Vector2d rotated(double angle, double xval, double yval) {
        double newX = xval * cos(angle) - yval * Math.sin(angle);
        double newY = xval * Math.sin(angle) + yval * cos(angle);
        return new Vector2d(newX, newY);
    }

    public void setDrivePower(PoseVelocity2d drivePower) {
        drive.setDrivePowers(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.localizer.getPose();
    }

    public void stop() {
        drive(0, 0, 0);
    }

}


