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
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
@Autonomous(name = "RedPlane", group = "Pushbot")
public class RedPlane extends LinearOpMode {
    public static double DISTANCE = 31.5; // in
    public static double toMid = 42.5;
    public static double toOtherSide = 25;
    public static double x = 48;
    public static double y = -34;
    public static double park = 25;
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
        Pose2d startPose = new Pose2d(-33, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .forward(DISTANCE)
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
                .forward(toMid)
                .turn(Math.toRadians(-90))
                .forward(toOtherSide)
                .splineTo(new Vector2d(x,-y), Math.toRadians(180))
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
                .strafeLeft(park)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .forward(29)
                .turn(Math.toRadians(-90))
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
                .strafeLeft(20)
                .forward(25)
                .splineTo(new Vector2d(48,-34), Math.toRadians(180))
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
                .strafeLeft(25)
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
//                .back(toBoard + leftReverse)
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
