/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TechnicalTerrors;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
;

import java.util.ArrayList;

@Autonomous(name = "Normal", group = "Pushbot")
public class Auto extends LinearOpMode {


    /* Declare OpMode members. */
    Hardware robot = new Hardware();

    private ElapsedTime runtime = new ElapsedTime();// Use a Pushbot's hardware

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR_HEX = 288;
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double SPOOL_DIAMETER_INCHES = 2.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_SPOOL = (COUNTS_PER_MOTOR_HEX * DRIVE_GEAR_REDUCTION) /
            (SPOOL_DIAMETER_INCHES * 3.14);

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    Orientation angles;

    private static final String VUFORIA_KEY =
            "AeWr4R//////AAABmbYsM7cHZkA+sRMzs/bFu44j3cYkmMaU6H+xbpPJQ9SynROk2nudJ0sVbgyn7Gc+hGMZ8rFyx0+8iSoQ8O1LZ5+V+4OcX1sL9z1puLugUplscZg7Q53zbPd3BnkON+tt6fFC0VNDNwUEACrMk+TM9szoroXvoJ5PYvywBFuVVq559PhuXwiobyzmMQI7Pb+gXlQj5EmvKn6etHHmWka2xLh1yq85NBuAFaVnlumgvaE+XQywedFTfZVGr6bI0iAvJLci37969ClMcWj1gPRHMq13paRiNoRfRm8HtkJgzP8WTPQIGeNxYSCV2qnppIACTLkxet8jIH6+evJ579rOhvxYxxd0ot7eDAtdfYQqqCPh";


    // Class Members


    @Override
    public void runOpMode() {
        // Initialize all motors, servos, and sensors
        robot.init(hardwareMap);

        // Set gyro angle
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);

        // Build April Tags Processor
        AprilTagProcessor myAprilTagProcessor;

        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()) // Get current april tags
                .setDrawTagID(true) // Show ID on steam
                .setDrawTagOutline(true) // Show outline on stream
                .setDrawAxes(true) // Show draw axes on stream
                .setDrawCubeProjection(true) // Show projection on stream
                .build();


        // Create the TensorFlow Processor with default settings (if change is needed see: https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/vision_processor_init/vision-processor-init.html#tensorflow-initialization-builder)
        TfodProcessor myTfodProcessor;

        myTfodProcessor = TfodProcessor.easyCreateWithDefaults();

        // Create the VisionPortal
        VisionPortal myVisionPortal;

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(robot.webcam)
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480)) // Size of stream
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // MJPEG is smaller uses less bandwidth, YUY2 is bigger
                .setAutoStopLiveView(true)
                .build();

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftForwardDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightForwardDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Display to phone when init is finished
        telemetry.addLine("Ready to start!");
        telemetry.update();


        waitForStart();



        robot.cameraServo.setPosition(-.2);
        ArrayList<AprilTagDetection> detections = myAprilTagProcessor.getDetections();
        if(!detections.isEmpty()) { // If it sees an April Tag
            for (AprilTagDetection detection : detections) {
                if (detection.id <= 6) { /** Alliance */
                    if(detection.id <= 3) { // Blue

                    } else { // Red

                    }

                } else { /** Audience */
                    if(detection.id <= 8) { // Red

                    } else { // Blue

                    }
                }
            }
        } else { // If it fails to see an April Tag

        }
    }


    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int newRForwardTarget;
        int newLForwardTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double leftFSpeed;
        double rightFSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() - moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;
            newRForwardTarget = robot.rightForwardDrive.getCurrentPosition() - moveCounts;
            newLForwardTarget = robot.leftForwardDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftForwardDrive.setTargetPosition(newLForwardTarget);
            robot.rightForwardDrive.setTargetPosition(newRForwardTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightForwardDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.

            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);
            robot.leftForwardDrive.setPower(speed);
            robot.rightForwardDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()
                            && robot.leftForwardDrive.isBusy() && robot.rightForwardDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;
                leftFSpeed = speed - steer;
                rightFSpeed = speed + steer;


                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                    leftFSpeed /= max;
                    rightFSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);
                robot.leftForwardDrive.setPower(leftFSpeed);
                robot.rightForwardDrive.setPower(rightFSpeed);


                // Display drive status for the driver.


            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightForwardDrive.setPower(0);
            robot.leftForwardDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroStrafe(double speed, double distance, double angle) {
        int newLeftTarget;
        int newRightTarget;
        int newRForwardTarget;
        int newLForwardTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double rightFSpeed;
        double leftFSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;
            newRForwardTarget = robot.rightForwardDrive.getCurrentPosition() + moveCounts;
            newLForwardTarget = robot.leftForwardDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftForwardDrive.setTargetPosition(newLForwardTarget);
            robot.rightForwardDrive.setTargetPosition(newRForwardTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightForwardDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);
            robot.leftForwardDrive.setPower(speed);
            robot.rightForwardDrive.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()
                            && robot.leftForwardDrive.isBusy() && robot.rightForwardDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;
                leftFSpeed = speed - steer;
                rightFSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                    leftFSpeed /= max;
                    rightFSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);
                robot.leftForwardDrive.setPower(leftFSpeed);
                robot.rightForwardDrive.setPower(rightFSpeed);
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightForwardDrive.setPower(0);
            robot.leftForwardDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftForwardDrive.setPower(0);
        robot.rightForwardDrive.setPower(0);
    }

    public void moveSlide(double speed, double slideInches, double timeoutS) {
        int spoolTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            spoolTarget = robot.slide1.getCurrentPosition() + (int) (slideInches * COUNTS_PER_INCH_SPOOL);
            robot.slide1.setTargetPosition(spoolTarget);


            // Turn On RUN_TO_POSITION
            robot.slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.slide1.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.slide1.isBusy())) {

                // Display it for the driver.
            /*    telemetry.addData("Path1",  "Running to %7d :%7d", spoolTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.spool.getCurrentPosition());
           */
            }

            // Stop all motion;
            robot.slide1.setPower(0);
            robot.slide1.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move
        }
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDrive.setPower(leftSpeed);
        robot.rightDrive.setPower(rightSpeed);
        robot.leftForwardDrive.setPower(leftSpeed);
        robot.rightForwardDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);


    }
}

