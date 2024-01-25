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

package org.firstinspires.ftc.teamcode.RoboHawks;

import static org.firstinspires.ftc.teamcode.RoboHawks.HardwareRH.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Objects;


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
@TeleOp(name="RHOpMode", group="Robot")
public class OpMode extends LinearOpMode {
    private boolean sean = false;
    private PIDController controller;
    private PIDController controller2;
    public static double p = 0.004, i = 0, d = 0.0001;
    public static double f = 0.11;
    public static double p2 = 0, i2 = 0, d2 = 0;
    public static double f2 = 0.05;

    public static int target = 0;
    public static int target2 = 0;
    private final double ticks_in_degrees = 1425.1 / 360;
    boolean plane = false;
    boolean stop = true;
    boolean prevA = false;
    boolean prevLeftBumper = false;
    boolean prevRightBumper = false;
    Directions hangMode = Directions.STOP;
    enum Directions {
        UP,
        DOWN,
        STOP
    }
    FtcDashboard dashboard = FtcDashboard.getInstance();
    org.firstinspires.ftc.teamcode.RoboHawks.HardwareRH robot = new org.firstinspires.ftc.teamcode.RoboHawks.HardwareRH();

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.


    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.init(hardwareMap);
            // Drive code off of GM-zero
            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).

            robot.leftForwardDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);


            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;

                controller.setPID(p, i, d);
                int armPos = robot.arm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
                double power = pid + ff;

                controller2.setPID(p2, i2, d2);
                int armPos2 = robot.slide.getCurrentPosition();
                double pid2 = controller2.calculate(armPos2, target2);
                double ff2 = Math.cos(Math.toRadians(-target2 / ticks_in_degrees)) * f2;
                double power2 = pid2 + ff2;

                // Arm Code is from: <insert link>
                // Drive Code is from: <insert link>


                if (gamepad2.left_stick_y != 0) {
                    robot.arm.setPower(-gamepad2.left_stick_y);
                    target = armPos;
                } else {
                    robot.arm.setPower(power);
                }

                if (gamepad2.right_stick_y != 0) {
                    robot.slide.setPower(gamepad2.right_stick_y);
                    target2 = armPos2;
                } else {
                    robot.slide.setPower(power2);
                }

                if(gamepad2.left_trigger > 0)
                    robot.claw.setPosition(clawOpen1);
                if(gamepad2.left_bumper)
                    robot.claw.setPosition(clawOpen2);
                if(gamepad2.right_bumper)
                    robot.claw.setPosition(clawClosed);


                if (gamepad1.a && gamepad1.a != prevA) {
                    plane = !plane;
                }
                if (gamepad1.right_bumper && gamepad1.right_bumper != prevRightBumper) {
                    if (Objects.requireNonNull(hangMode) == Directions.STOP) {
                        hangMode = Directions.DOWN;
                    } else {
                        hangMode = Directions.STOP;
                    }
                }
                if (gamepad1.left_bumper && gamepad1.left_bumper!= prevLeftBumper) {
                    if (Objects.requireNonNull(hangMode) == Directions.STOP) {
                        hangMode = Directions.UP;
                    } else {
                        hangMode = Directions.STOP;
                    }
                }

                if(gamepad1.left_trigger > 0) {
                    robot.hangRotate.setPower(gamepad1.left_trigger);
                }
                if(gamepad1.right_trigger > 0) {
                    robot.hangRotate.setPower(gamepad1.right_trigger);
                }

                switch(hangMode) {
                    case UP:
                        robot.hangSpin.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.hangSpin.setPower(1);
                        break;
                    case DOWN:
                        robot.hangSpin.setDirection(DcMotorSimple.Direction.REVERSE);
                        robot.hangSpin.setPower(1);
                        break;
                    case STOP:
                        robot.hangSpin.setPower(0);
                        break;
                    }
                if(plane) {
                    robot.plane.setPower(1);
                }
                if(!plane) {
                    robot.plane.setPower(0);
                }


                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,

                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;
                if(gamepad1.left_stick_y != 0) {
                    robot.leftForwardDrive.setPower(-frontLeftPower);
                    robot.leftDrive.setPower(backLeftPower);
                    robot.rightForwardDrive.setPower(-frontRightPower);
                    robot.rightDrive.setPower(backRightPower);
                } else {
                    robot.leftForwardDrive.setPower(frontLeftPower);
                    robot.leftDrive.setPower(backLeftPower);
                    robot.rightForwardDrive.setPower(frontRightPower);
                    robot.rightDrive.setPower(backRightPower);
                }

                prevA = gamepad1.a;
                prevLeftBumper = gamepad1.left_bumper;
                prevRightBumper = gamepad1.right_bumper;
                telemetry.addData("target", target);
                telemetry.addData("target2", target2);
                telemetry.addData("arm", robot.arm.getCurrentPosition());
                telemetry.addData("test1", "test1");
                updateTelemetry(telemetry);
            }
        }
    }
}
