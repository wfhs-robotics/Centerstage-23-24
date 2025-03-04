package org.firstinspires.ftc.teamcode.TechnicalTerrors.Autonomous;

import static org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware.*;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware;
import org.firstinspires.ftc.teamcode.TechnicalTerrors.PoseStorage;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "BlueBoard", group = "Pushbot")
public class BlueBoard extends LinearOpMode {
    public static double DISTANCE = 36.5; // in
    public static double toBoard = 25;
    public static double wristNum= .45;
    public static double rightReverse = 42.5;
    public static double leftReverse = 2;
    public static double dis1 = 18;
    public static double dis2 = 25;
    public static double rightBoardStrafe = 13;
    public static double parkDistance = 15;
    /* Flags, one for spike for dev mode, PIXELPOS is to be updated before every match. */
    public static String spike = "Left";
    public static String PIXELPOS = "Left";
    /* Default strafe to the left side. */
    public static double middleStrafe = 8;
    public static double leftStrafe = 2;
    public static double rightStrafe = 4;

    /* All Board stacking positions. <spike><boardPos>Y */
    public static double middleLeftStrafe = 6;
    public static double middleRightStrafe = 8;
    public static double rightLeftStrafe = 1;
    public static double rightRightStrafe = 0.1;
    public static double leftLeftStrafe = 7;
    public static double leftRightStrafe = 5;
    public static boolean dev = false;
    double cX = 0;
    double cY = 0;
    double width = 0;
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static int CAMERA_WIDTH = 960; // width  of wanted camera resolution
    private static int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Hardware robot =  new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        if (PIXELPOS.equals("Left")) {
            middleStrafe =  middleLeftStrafe;
            leftStrafe = leftLeftStrafe;
            rightStrafe = rightLeftStrafe;
            telemetry.addData("Pixel Position: ", "Left");
        } else {
            middleStrafe = middleRightStrafe;
            leftStrafe = leftRightStrafe;
            rightStrafe = rightRightStrafe;
            telemetry.addData("Pixel Position: ", "Right");
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14, 62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .forward(31.5)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawOpen1);
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(robot.arm1.getPosition() - (onePixel * 2));
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(robot.arm2.getPosition() + (onePixel * 2));
                })

                .back(6)
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawClosed);
                })
                .turn(Math.toRadians(-90))
                .back(42.5)
                .strafeLeft(middleStrafe)
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
                .strafeRight(25)
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .forward(36.5)
                .turn(Math.toRadians(-90))
                .forward(1)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawOpen1);
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(robot.arm1.getPosition() - (onePixel * 2));
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(robot.arm2.getPosition() + (onePixel * 2));
                })

                .back(1)
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawClosed);
                })
                .back(41.5 + 2)
                .strafeRight(rightStrafe)
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
                .forward(4)
                .strafeRight(28)
                .back(2)
                .build();
                /*
                .forward(31.5)
                .turn(Math.toRadians(-90))
                .forward(5)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawOpen1);
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(robot.arm1.getPosition() - onePixel );
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(robot.arm2.getPosition() + onePixel);
                })
                .back(toBoard)
                .strafeLeft(rightStrafe)
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
                .strafeRight(parkDistance)
                .back(2)
                .build();

                 */


        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .forward(5)
                .strafeLeft(18)
                .forward(28)
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawOpen1);
                })
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    robot.arm1.setPosition(robot.arm1.getPosition() - (onePixel * 2));
                })
                .addTemporalMarker(() -> {
                    robot.arm2.setPosition(robot.arm2.getPosition() + (onePixel * 2));
                })

                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(clawClosed);
                })
                .back(21)
                .strafeRight(leftStrafe)
                .back(4)
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
                .strafeRight(18)
                .back(2)
                .build();


        initOpenCV();
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        waitForStart();
        if (isStopRequested()) return;

        if(!dev) {
            if (cX >= 0 && cX < 400) {
                telemetry.addData("Status", "Left");
                telemetry.update();
                drive.followTrajectorySequence(left);
            } else if (cX >= 400 && cX < 800) {
                telemetry.addData("Status", "Middle");
                telemetry.update();
                drive.followTrajectorySequence(middle);

            } else if(cX >= 800) {
                telemetry.addData("Status", "Right");
                telemetry.update();
                drive.followTrajectorySequence(right);
            } else {
                drive.followTrajectorySequence(middle);
                telemetry.addData("Status", "Detected no element, running middle path.");
            }
        } else {
            if (spike.equals("Middle"))
                drive.followTrajectorySequence(middle);
            else if (spike.equals("Right"))
                drive.followTrajectorySequence(right);
            else if (spike.equals("Left"))
                drive.followTrajectorySequence(left);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new DetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class DetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerRed = new Scalar(100, 100, 100);
            Scalar upperRed = new Scalar(180, 255, 255);

            Scalar lowerBlue = new Scalar(0, 50, 50);
            Scalar upperBlue = new Scalar(80, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

}
