package org.firstinspires.ftc.teamcode.TechnicalTerrors.Autonomous;

import static org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware.clawOpen1;
import static org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware.clawOpen2;
import static org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware.onePixel;
import static org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware.wristInside;
import static org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware.yPos1;
import static org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware.yPos2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware;
import org.firstinspires.ftc.teamcode.TechnicalTerrors.PoseStorage;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "BluePlane", group = "Pushbot")
public class BluePlane extends LinearOpMode {
    public static double DISTANCE = 36.5; // in
    public static double toBoard = 42.5;
    public static double wristNum= .45;
    public static String spike = "Left";
    public static double rightReverse = 20.5;
    public static double leftReverse = 2;
    public static double rightBoardStrafe = 13;
    public static double parkDistance = 31;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Hardware robot =  new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .forward(31.5)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawOpen1);
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(robot.arm1.getPosition() - onePixel);
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(robot.arm2.getPosition() + onePixel);
                })
                .back(6)
                .turn(Math.toRadians(90))
                .back(42.5)
                .strafeRight(4)
                .addTemporalMarker(() -> {
                   robot.arm1.setPosition(yPos1);
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(yPos2);
                })
                .addTemporalMarker(() -> {
                    robot.wrist.setPosition(wristNum);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawOpen2);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(yPos2);
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(yPos1);
                })
                .addTemporalMarker(() -> {
                    robot.wrist.setPosition(wristInside);
                })
                .forward(2)
                .strafeLeft(parkDistance)
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .forward(36.5)
                .turn(Math.toRadians(90))
                .back(20.5)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawOpen1);
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(robot.arm1.getPosition() - onePixel);
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(robot.arm2.getPosition() + onePixel);
                })
                .back(toBoard - 20.5)
                .strafeLeft(rightBoardStrafe)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(yPos1);
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(yPos2);
                })
                .addTemporalMarker(() -> {
                    robot.wrist.setPosition(wristNum);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawOpen2);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(yPos2);
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(yPos1);
                })
                .addTemporalMarker(() -> {
                    robot.wrist.setPosition(wristInside);
                })
                .forward(2)
                .strafeLeft(parkDistance)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .forward(DISTANCE)
                .turn(Math.toRadians(90))
                .forward(leftReverse)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawOpen1);
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(robot.arm1.getPosition() - onePixel);
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(robot.arm2.getPosition() + onePixel);
                })
                .back(toBoard + leftReverse)
                .strafeRight(2)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(yPos1);
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(yPos2);
                })
                .addTemporalMarker(() -> {
                    robot.wrist.setPosition(wristNum);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawOpen2);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(yPos2);
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(yPos1);
                })
                .addTemporalMarker(() -> {
                    robot.wrist.setPosition(wristInside);
                })
                .strafeLeft(31)
                .forward(2)
                .build();



        waitForStart();

        if (isStopRequested()) return;
        if(spike.equals("Middle"))
            drive.followTrajectorySequence(middle);
        else if(spike.equals("Right"))
            drive.followTrajectorySequence(right);
        else if(spike.equals("Left"))
            drive.followTrajectorySequence(left);

        PoseStorage.currentPose = drive.getPoseEstimate();
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        while (!isStopRequested() && opModeIsActive()) ;
    }

}
