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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
@Config
public class Hardware
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftForwardDrive    = null;
    public DcMotor  rightForwardDrive    = null;
    public Servo cameraServo = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    public DcMotor slide1 = null;
    public DcMotor slide2 = null;
    public Servo claw = null;
    public Servo wrist = null;
    public WebcamName webcam = null;
    public BNO055IMU imu = null;
    public CRServo plane = null;

    /* Variables used to be in Strafe */
    public static double onePixel = .02;
    public static double aPos = .65;
    public static double yPos1 = 0;
    public static double yPos2 = .525;
    public static double wristOutside = .55;
    public static double wristInside = 0.015;
    public static double clawClosed = .9;
    public static double clawOpen1 = .835;
    public static double clawOpen2 = .725;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
        leftForwardDrive    = hwMap.get(DcMotor.class, "leftForwardDrive");
        rightForwardDrive = hwMap.get(DcMotor.class, "rightForwardDrive");
//        cameraServo = hwMap.get(Servo.class, "cameraServo");
        arm1 = hwMap.get(Servo.class, "arm1");
        arm2 = hwMap.get(Servo.class, "arm2");
        wrist = hwMap.get(Servo.class, "wrist");
        claw = hwMap.get(Servo.class, "claw");
        slide1 = hwMap.get(DcMotor.class, "slide1");
        slide2 = hwMap.get(DcMotor.class, "slide2");
        plane = hwMap.get(CRServo.class, "plane");
//        webcam = hwMap.get(WebcamName.class, "Webcam 1");


        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftForwardDrive.setPower(0);
        rightForwardDrive.setPower(0);

        wrist.setPosition(.015);
        arm1.setPosition(.98);
        arm2.setPosition(.02);
        claw.setPosition(clawClosed);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftForwardDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightForwardDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        // Define and initialize ALL installed servos.

    }
}
