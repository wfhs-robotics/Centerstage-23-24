package org.firstinspires.ftc.teamcode.TechnicalTerrors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "RRTest", group = "Pushbot")
public class AutoRRTest extends LinearOpMode {
    public static double DISTANCE = 31.5; // in
    public static double toBoard = 42;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Hardware robot =  new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        TrajectorySequence redAlliance = drive.trajectorySequenceBuilder(startPose)
                .forward(DISTANCE)
                .turn(Math.toRadians(90))
                .back(toBoard)
                .build();



        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(redAlliance);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
