package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class wristTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Wrist starting position
        int wristDownPosition = 0;

        // Position of the arm when it's on the ground
        int wristUpPosition = 145;


        // Position of the arm when it's on the backboard
        // int wristDownPosition = 0;

        // Find a motor in the hardware map named "Arm Motor"
        DcMotor wristMotor = hardwareMap.dcMotor.get("wristMotor");

        // Reset the motor encoder so that it reads zero ticks
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position
        wristMotor.setTargetPosition(wristDownPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // If the A button is pressed, raise the arm
            if (gamepad1.x) {
                wristMotor.setTargetPosition(wristUpPosition);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.5);
            }

            // If the B button is pressed, lower the arm
            if (gamepad1.y) {
                wristMotor.setTargetPosition(wristDownPosition);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.5);
            }

            // Get the current position of the wristMotor
            double position = wristMotor.getCurrentPosition();

            // Get the target position of the wristMotor
            double desiredPosition = wristMotor.getTargetPosition();

            // Show the position of the wristMotor on telemetry
            telemetry.addData("Encoder Position", position);

            // Show the target position of the wristMotor on telemetry
            telemetry.addData("Desired Position", desiredPosition);

            telemetry.update();
        }
    }
}