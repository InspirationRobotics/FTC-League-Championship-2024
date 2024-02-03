package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
 * SERVICES; LOSS OF USE, DATA, OR PROFITS.74; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

@TeleOp(name="RR Teleop")
public class RR_Teleop extends LinearOpMode {

    //defining motors and servos
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor wristMotor = null;
    public DcMotor armMotor = null;
    public Servo drone = null;
    public Servo clawLeft = null;
    public Servo clawRight = null;


    //variables (positions)

    // Wrist depositing on backdrop position (ticks):
    int wristDepositingPosition = -91;

    // Position of the wrist when it's on the ground (ticks)
    int wristDownPosition = 138;

    // Position of the arm when it's lifted (ticks):
    int armUpPosition = 1900;

    // Position of the arm when it's init position (ticks):
    int armIntPosition = 0;

    // Position of the wrist when it's init position (ticks):
    int wristIntPosition = 0;

    int armHangPosition = 1450;

    double leftFrontSpeed = 0;
    double leftBackSpeed = 0;
    double rightFrontSpeed = 0;
    double rightBackSpeed = 0;
    int dronePressTimes = 0;

    public void runOpMode() {
        // Insert whatever initialization your own code does
        wristMotor = hardwareMap.get(DcMotor.class, "wristMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        drone = hardwareMap.get(Servo.class, "drone");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");// ctrl 0
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");// ctrl 1
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");// ctrl 2
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");// ctrl 3

        // Assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

        waitForStart();

        while (opModeIsActive()) {
            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.update();

            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());

            // Insert whatever teleop code you're using
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin((theta) - Math.PI * 5/4);
            double cos = Math.cos((theta) - Math.PI * 5/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            leftFrontSpeed = power * sin/max + turn;
            leftBackSpeed = power * cos/max + turn;
            rightBackSpeed = power * sin/max - turn;
            rightFrontSpeed = power * cos/max - turn;

            if ((power + Math.abs(turn)) > 1) {
                leftFrontSpeed /= power + Math.abs(turn);
                leftBackSpeed /= power + Math.abs(turn);
                rightBackSpeed /= power + Math.abs(turn);
                rightFrontSpeed /= power + Math.abs(turn);
            }

            leftFront.setPower(leftFrontSpeed);
            leftBack.setPower(leftBackSpeed);
            rightFront.setPower(rightFrontSpeed);
            rightBack.setPower(rightBackSpeed);

            telemetry.addData("Angle", theta * 57.296);
            telemetry.addData("Power", power);
            telemetry.addData("Left Front", leftFrontSpeed);
            telemetry.addData("Left Back", leftBackSpeed);
            telemetry.addData("Right Back", rightBackSpeed);
            telemetry.addData("Right Front", rightFrontSpeed);

            // If the dpad_up is pressed, lift the arm
            if (gamepad2.dpad_up) {
                armMotor.setTargetPosition(armUpPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
                wristMotor.setTargetPosition(wristDepositingPosition);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(1);

            }
            // If the dpad_down is pressed, lower the arm
            if (gamepad2.dpad_down) {
                armMotor.setTargetPosition(armIntPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
                wristMotor.setTargetPosition(wristDownPosition);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.5);
            }

            if (gamepad2.x) {
                armMotor.setTargetPosition(armIntPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
                wristMotor.setTargetPosition(wristIntPosition);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(1);
            }

            if (gamepad2.y) {
                armMotor.setTargetPosition(armHangPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
                wristMotor.setTargetPosition(wristIntPosition);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(1);
            }

            //claws
            if (gamepad2.left_bumper)
                clawLeft.setPosition(0.3); //open left claw
            if (gamepad2.right_bumper)
                clawRight.setPosition(0.7); //open right claw
            if (gamepad2.left_trigger > 0.5)
                clawLeft.setPosition(0); //close left claw
            if (gamepad2.right_trigger > 0.5)
                clawRight.setPosition(0.95); //close right claw

            // Drone
            if(gamepad2.a) {
                dronePressTimes += 1;
                if (dronePressTimes >= 2)
                    drone.setPosition(0.7);
            }
            if(gamepad2.b)
                drone.setPosition(0);
            }

            // Get the current position of the armMotor
            double armPosition = armMotor.getCurrentPosition();
            // Get the target position of the armMotor
            double armDesiredPosition = armMotor.getTargetPosition();
            // Show the position of the armMotor on telemetry
            telemetry.addData(" Arm Encoder Position", armPosition);
            // Show the target position of the armMotor on telemetry
            telemetry.addData("Arm Desired Position", armDesiredPosition);
            telemetry.update();

            // Get the current position of the wristMotor
            double wristPosition = wristMotor.getCurrentPosition();

            // Get the target position of the wristMotor
            double wristDesiredPosition = wristMotor.getTargetPosition();

            // Show the position of the wristMotor on telemetry
            telemetry.addData("Wrist Encoder Position", wristPosition);

            // Show the target position of the wristMotor on telemetry
            telemetry.addData("Wrist Desired Position", wristDesiredPosition);

            telemetry.update();
            }
        }