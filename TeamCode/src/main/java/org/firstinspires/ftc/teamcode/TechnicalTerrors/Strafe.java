/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;


/**
 * This OpMode Sample illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy ConceptExternalHardwareClass.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear opmode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, drawing from this Sample; select TeleOp.
 *  Also add another new file named RobotHardware.java, drawing from the Sample with that name; select Not an OpMode.
 */

@Config
@TeleOp(name="Strafe", group="Robot")
public class Strafe extends LinearOpMode {
    private PIDController controller;
    private PIDController controller2;
    public static double p = 0.004, i = 0, d = 0.0001;
    public static double f = 0.11;
    public static double p2 = 0.004, i2 = 0, d2 = 0.0001;
    public static double f2 = 0.11;

    public static int target = 0;
    public static int target2 = 0;
    private final double ticks_in_degrees = 1425.1 / 360;
    private boolean sean = false;


    public int driveDirection = 1;

    boolean prevDDown = false;
    boolean prevDUp = false;
    boolean plane = false;
    String high = "null";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware robot = new org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware();


    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.arm1.setPosition(1.0);
        robot.arm2.setPosition(0);
        target = 0;
//        robot.leftForwardDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            controller.setPID(p, i, d);
            int armPos = robot.slide1.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double power = pid + ff;

            controller2.setPID(p2, i2, d2);
            int armPos2 = robot.slide2.getCurrentPosition();
            double pid2 = controller2.calculate(armPos2, -target);
            double ff2 = Math.cos(Math.toRadians(-target / ticks_in_degrees)) * f2;
            double power2 = pid2 + ff2;


            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed, but driving was reversed so I un negatived it
            double x = -gamepad1.left_stick_x * 1.1; // Negative because stafing was reversed
            double rx = -gamepad1.right_stick_x;


            // Drive code off of GM-zero
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();


            if (gamepad2.left_stick_y != 0) {
                robot.slide1.setPower(-gamepad2.left_stick_y);
                robot.slide2.setPower(gamepad2.left_stick_y * 0.975);
                target = armPos;
                //target2 = armPos2;
            } else {
                robot.slide1.setPower(power);
                robot.slide2.setPower(power2);
            }


            if (gamepad2.x) {
                if (plane) {
                    plane = false;
                } else {
                    plane = true;
                }
            }
            if (plane) {
                robot.plane.setPower(1);
            } else {
                robot.plane.setPower(0);
            }

            if (gamepad2.left_bumper) {
                robot.claw.setPosition(robot.clawOpen1);
            }
            if (gamepad2.left_trigger > 0) {
                robot.claw.setPosition(robot.clawOpen2);
            }
            if (gamepad2.right_bumper) { // Close claw
                robot.claw.setPosition(robot.clawClosed);
            }
            if (gamepad2.a) { // Move arm to the inside
                robot.arm1.setPosition(.96);
                robot.arm2.setPosition(.04);
                robot.wrist.setPosition(robot.wristInside);
            }
            if (gamepad2.y) { // Move arm to the outside
                robot.arm1.setPosition(robot.yPos1);
                robot.arm2.setPosition(robot.yPos2);
                robot.wrist.setPosition(robot.wristOutside);
            }
            // Move down one pixel
            if (gamepad2.dpad_down && gamepad2.dpad_down != prevDDown && gamepad2.right_stick_y == 0) {
                dashboard.getTelemetry().addData("arm1Pos ", robot.arm1.getPosition());
                dashboard.getTelemetry().addData("arm2Pos ", robot.arm2.getPosition());
                dashboard.getTelemetry().update();
                robot.arm1.setPosition(robot.arm1.getPosition() + robot.onePixel);
                robot.arm2.setPosition(robot.arm2.getPosition() - robot.onePixel);
            }
            // Move up one pixel
            if (gamepad2.dpad_up && gamepad2.dpad_up != prevDUp) {
                dashboard.getTelemetry().addData("arm1Pos ", robot.arm1.getPosition());
                dashboard.getTelemetry().addData("arm2Pos ", robot.arm2.getPosition());
                dashboard.getTelemetry().update();
                robot.arm1.setPosition(robot.arm1.getPosition() - robot.onePixel);
                robot.arm2.setPosition(robot.arm2.getPosition() + robot.onePixel);
            }

            prevDDown = gamepad2.dpad_down;
            prevDUp = gamepad2.dpad_up;
            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            updateTelemetry(telemetry);
        }

    }
}




