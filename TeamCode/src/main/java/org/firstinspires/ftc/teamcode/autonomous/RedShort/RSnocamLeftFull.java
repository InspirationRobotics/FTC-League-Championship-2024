package org.firstinspires.ftc.teamcode.autonomous.RedShort;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RSnocamLeftFull extends LinearOpMode {
    //RIGHT

    public Servo clawLeft = null;
    public Servo clawRight = null;
    public DcMotor wristMotor = null;
    public DcMotor armMotor = null;

    int wristDownPosition = -91;
    // Position of the wrist when it's on the ground (ticks)
    int wristUpPosition = 145;
    // Position of the arm when it's lifted (ticks)
    int armUpPosition = 1900;
    // Position of the arm when it's down (ticks)
    int armDownPosition = 0;



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        wristMotor = hardwareMap.get(DcMotor.class, "wristMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        // Reset the motor encoder so that it reads zero ticks
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        //claw down
        wristMotor.setTargetPosition(138);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(0.3);
        //drive to left spike mark
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(3)
                .forward(30)
                .turn(Math.toRadians(92))
                .forward(3.5)
                .build();
        drive.followTrajectorySequence(trajSeq);
        //drop purple pixel
        clawLeft.setPosition(0.3);
        //back up to the backdrop and strafe to left april tag
        trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .waitSeconds(2)
                .back(27)
                //.strafeRight(6)
                .build();
        drive.followTrajectorySequence(trajSeq);
        trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeRight(6)
                .build();
        drive.followTrajectorySequence(trajSeq);
        //go to dropping position
        clawLeft.setPosition(0);
        sleep(500);
        armMotor.setTargetPosition(2075);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
        wristMotor.setTargetPosition(-91);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(0.3);
        //wait for motion to complete
        trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .waitSeconds(3)
                .back(8)
                .waitSeconds(1)
                .build();
        drive.followTrajectorySequence(trajSeq);
        //open the right claw to drop the yellow pixel
        clawRight.setPosition(0.7);
        //back up and strafe to parking position
        trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .waitSeconds(2)
                .forward(10)
                .strafeLeft(31)
                .build();
        drive.followTrajectorySequence(trajSeq);
        //close up (init position)
        clawRight.setPosition(0.95);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
        sleep(1000);
        wristMotor.setTargetPosition(0);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(0.3);
        //drive in and park
        trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(26)
                .build();
        drive.followTrajectorySequence(trajSeq);
        if(isStopRequested()) return;
    }
}