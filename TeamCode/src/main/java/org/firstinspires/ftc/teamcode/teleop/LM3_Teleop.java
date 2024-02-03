package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="LM3 Teleop")
public class LM3_Teleop extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;

    public DcMotor wristMotor = null;
    public DcMotor armMotor = null;

    public Servo drone = null;
    public Servo clawLeft = null;
    public Servo clawRight = null;


    HardwareMap hwMap = null;

    // Wrist starting position (ticks)
    int wristDownPosition = -91;
    // Position of the wrist when it's on the ground (ticks)
    int wristUpPosition = 142;
    // Position of the arm when it's lifted (ticks)
    int armUpPosition = 1900;
    // Position of the arm when it's down (ticks)
    int armDownPosition = 0;

    int armIntPosition = 0;

    int wristIntPosition = 0;

    double leftFrontSpeed = 0;
    double leftBackSpeed = 0;
    double rightFrontSpeed = 0;
    double rightBackSpeed = 0;
    int dronePressTimes = 0;

    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        // Save reference to Hardware map
        // Define and Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");// ctrl 0
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");// ctrl 1
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");// ctrl 2
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");// ctrl 3

        wristMotor = hardwareMap.get(DcMotor.class, "wristMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        drone = hardwareMap.get(Servo.class, "drone");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");


        telemetry.addData("Say", "Hello Driver");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the motor encoder so that it reads zero ticks
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFrontSpeed = gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger;
        leftBackSpeed = gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger;
        rightBackSpeed = gamepad1.right_stick_y + gamepad1.left_trigger - gamepad1.right_trigger;
        rightFrontSpeed = gamepad1.right_stick_y - gamepad1.left_trigger + gamepad1.right_trigger;

        if (leftFrontSpeed > 1)
            leftFrontSpeed = 1;
        if (leftFrontSpeed < -1)
            leftFrontSpeed = -1;
        if (rightFrontSpeed > 1)
            rightFrontSpeed = 1;
        if (rightFrontSpeed < -1)
            rightFrontSpeed = -1;
        if (leftBackSpeed > 1)
            leftBackSpeed = 1;
        if (leftBackSpeed < -1)
            leftBackSpeed = -1;
        if (rightBackSpeed > 1)
            rightBackSpeed = 1;
        if (rightBackSpeed < -1)
            rightBackSpeed = -1;
        leftBack.setPower(-leftFrontSpeed);
        rightBack.setPower(-rightFrontSpeed);
        leftFront.setPower(-leftBackSpeed);
        rightFront.setPower(-rightBackSpeed);


        // If the x button is pressed, lift the wrist
        //if (gamepad2.x) {
        //wristMotor.setTargetPosition(wristUpPosition);
        // wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //wristMotor.setPower(1);
        //}

        // If the y button is pressed, lower the wrist
        //if (gamepad2.y) {
        //wristMotor.setTargetPosition(wristDownPosition);
        // wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // wristMotor.setPower(1);
        // }

        // If the dpad_up is pressed, lift the arm
        if (gamepad2.dpad_up) {
            armMotor.setTargetPosition(armUpPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);
            wristMotor.setTargetPosition(wristDownPosition);
            wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristMotor.setPower(1);

        }
        // If the dpad_down is pressed, lower the arm
        if (gamepad2.dpad_down) {
            armMotor.setTargetPosition(armDownPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);
            wristMotor.setTargetPosition(wristUpPosition);
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
        //run
        if (gamepad2.a) {
                drone.setPosition(1);
        }
        //reset
        if (gamepad2.b)
            drone.setPosition(0.9);

            // Manual arm movement for hanging
        if (gamepad2.left_stick_y != 0) {
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armMotor.setPower(gamepad2.left_stick_y);
            }

            // Get the current position of the armMotor
            double armPosition = armMotor.getCurrentPosition();
            // Get the target position of the armMotor
            double armDesiredPosition = armMotor.getTargetPosition();
            // Show the position of the armMotor on telemetry
            telemetry.addData("Encoder Position", armPosition);
            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position", armDesiredPosition);
            telemetry.update();

            // Get the current position of the wristMotor
            double wristPosition = wristMotor.getCurrentPosition();

            // Get the target position of the wristMotor
            double wristDesiredPosition = wristMotor.getTargetPosition();

            // Show the position of the wristMotor on telemetry
            telemetry.addData("Encoder Position", wristPosition);

            // Show the target position of the wristMotor on telemetry
            telemetry.addData("Desired Position", wristDesiredPosition);

            telemetry.update();
        }
    }
