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
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RoboHawks.Hardware;


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


@TeleOp(name="NoRRTest", group="Robot")
public class NoRR extends LinearOpMode {
    private boolean sean = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware robot = new org.firstinspires.ftc.teamcode.TechnicalTerrors.Hardware();

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.


    @Override
    public void runOpMode() {
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.init(hardwareMap);
            // Drive code off of GM-zero
            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.0; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double normalTurn =  gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            if (gamepad1.dpad_down) {
                sean = true;
            }
            if (gamepad1.dpad_up) {
                sean = false;
            }

//            if(gamepad2.dpad_down){
//                robot.slide1.setPosition(0);
//                robot.slide2.setPosition(0);
//            }
//
//            if(gamepad2.dpad_up){
//                robot.slide1.setPosition(.1);
//                robot.slide.setPosition(.1);
//            }
//
//            if (gamepad2.left_bumper){
//                robot.slide1.setPosition(.2);
//                robot.slide2.setPosition(.2);
//            }
//
//            if (gamepad2.dpad_right){
//                robot.slide1.setPosition(1);
//                robot.arm2.setPosition(1);
//            }
//
//            if (gamepad2.left_trigger > 0){
//                robot.arm1.setPosition(.4);
//                robot.arm2.setPosition(.4);
//            }

//            if (gamepad2.right_bumper){
//                robot.claw.setPosition(.3);
//            }
//
//            if (gamepad2.right_trigger > 0){
//                robot.claw.setPosition(0);
//            }
//
//            if (gamepad2.a){
//                robot.claw.setPosition(1);
//            }


            if (normalTurn == 0 && !sean) {
                telemetry.addData(">", "Strafe");
                telemetry.update();
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Strafe", true);
                dashboard.sendTelemetryPacket(packet);
                robot.leftForwardDrive.setPower(leftFrontPower);
                robot.leftDrive.setPower(leftBackPower);
                robot.rightForwardDrive.setPower(rightFrontPower);
                robot.rightDrive.setPower(rightBackPower);
            } else if (!sean) { // If turning without sean mode
                telemetry.addData(">", "turn");
                telemetry.update();
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("rightStickX", rx);
                packet.put("normalTurn", normalTurn);
                packet.put("Strafe", false);
                dashboard.sendTelemetryPacket(packet);
                leftBackPower = Range.clip(normalTurn, -1.0, 1.0);
                rightBackPower = Range.clip(normalTurn, -1.0, 1.0);
                rightFrontPower = Range.clip(normalTurn, -1.0, 1.0);
                leftFrontPower = Range.clip(normalTurn, -1.0, 1.0);


                robot.leftForwardDrive.setPower(-leftFrontPower);
                robot.rightForwardDrive.setPower(-rightFrontPower);
                robot.leftDrive.setPower(leftBackPower);
                robot.rightDrive.setPower(rightBackPower);
            } else { // If sean mode is on
                // Cut power 20%
                telemetry.addData(">", "Sean Mode: On");
                telemetry.update();
                robot.leftDrive.setPower(robot.leftDrive.getPower() * 0.2);
                robot.rightDrive.setPower(robot.rightDrive.getPower() * 0.2);
                robot.leftForwardDrive.setPower(robot.leftForwardDrive.getPower() * 0.2);
                robot.rightForwardDrive.setPower(robot.rightForwardDrive.getPower() * 0.2);
                if (gamepad1.right_stick_x > 0) {
                    // Turn at 50%
                    robot.leftDrive.setPower(Range.clip(normalTurn, -.5, .5));
                    robot.rightDrive.setPower(Range.clip(normalTurn, -.5, .5));
                    robot.leftForwardDrive.setPower(Range.clip(normalTurn, -.5, .5));
                    robot.rightForwardDrive.setPower(Range.clip(normalTurn, -.5, .5));
                }
            }
            ;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("leftFrontPower", leftFrontPower);
            packet.put("leftBackPower", leftBackPower);
            packet.put("rightFrontPower", rightFrontPower);
            packet.put("rightBackPower", rightBackPower);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
