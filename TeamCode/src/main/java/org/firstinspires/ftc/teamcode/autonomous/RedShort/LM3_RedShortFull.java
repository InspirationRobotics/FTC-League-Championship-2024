/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.autonomous.RedShort;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class LM3_RedShortFull extends LinearOpMode
{
    OpenCvWebcam webcam;

    final int LEFT = 1;
    final int CENTER = 2;
    final int RIGHT = 3;

    int pixelLocation = 0;

    public Servo clawLeft = null;
    public Servo clawRight = null;
    public DcMotor wristMotor = null;
    public DcMotor armMotor = null;

    int wristGroundPosition = 138;
    int wristInitialPosition = 0;
    int wristDepositPosition = -91;
    int armUpPosition = 1950;
    int armDownPosition = 0;
    double clawLeftOpenPosition = 0.3;
    double clawLeftClosePosition = 0;
    double clawRightOpenPosition = 0.7;
    double clawRightClosePosition = 1;


    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        wristMotor = hardwareMap.get(DcMotor.class, "wristMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new SamplePipeline());

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

                clawLeft.setPosition(0);
                clawRight.setPosition(0.95);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();


        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            if (pixelLocation == CENTER) {
                moveWristToGround(0.5);
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .waitSeconds(2)
                        .forward(27)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                openClawLeft();
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .waitSeconds(1)
                        .back(23)
                        .build();
                drive.followTrajectorySequence(trajSeq);
               closeClawLeft();
                moveWristToInit(0.5);
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .waitSeconds(1)
                        .strafeRight(24)
                        .forward(23)
                        .turn(Math.toRadians(92))
                        .build();
                drive.followTrajectorySequence(trajSeq);
                moveWristToDeposit(0.7);
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(3)
                        .waitSeconds(3)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                openClawRight();
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .forward(10)
                        .strafeLeft(27)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                closeClawRight();
               moveArmDown(0.7);
                sleep(1000);
                moveWristToInit(0.7);
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(25.5)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                break;
            }

            if (pixelLocation == LEFT) {
               moveWristToGround(0.5);
                //drive to left spike mark
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .waitSeconds(1.5)
                        .forward(30)
                        .turn(Math.toRadians(92))
                        .forward(4.5)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                //drop purple pixel
                openClawLeft();
                //back up to the backdrop and strafe to left april tag
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .waitSeconds(1)
                        .back(20)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                //go to dropping position
                closeClawLeft();
                moveArmUp(0.7);
                sleep(500);
                moveWristToDeposit(0.7);
                //wait for motion to complete
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .waitSeconds(1)
                        .back(10)
                        .strafeRight(2)
                        .waitSeconds(1)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                //open the right claw to drop the yellow pixel
                openClawRight();
                //back up and strafe to parking position
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .waitSeconds(1)
                        .forward(10)
                        .strafeLeft(30)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                //close up (init position)
                closeClawRight();
                moveArmDown(0.7);
                sleep(1000);
                moveWristToInit(0.7);
                //drive in and park
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(24)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                break;
            }

            if (pixelLocation == RIGHT) {
                moveWristToGround(0.7);
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .waitSeconds(1.5)
                        .forward(24)
                        .strafeRight(11.5)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                openClawLeft();
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .waitSeconds(0.5)
                        .back(8.5)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                closeClawLeft();
                moveWristToInit(0.5);
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .waitSeconds(1)
                        .strafeRight(19)
                        .turn(Math.toRadians(92))
                        .strafeLeft(2)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                moveArmUp(0.7);
                moveWristToDeposit(0.7);
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(2)
                        .waitSeconds(3)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                openClawRight();
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .waitSeconds(0.5)
                        .forward(10)
                        .strafeLeft(18)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                closeClawRight();
                moveArmDown(0.7);
                sleep(1000);
               moveWristToInit(0.7);
                trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(25.5)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                break;
            }



            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    public void openClawLeft() {
        clawLeft.setPosition(clawLeftOpenPosition);
        telemetry.addLine("Opened left claw");
    }
    public void closeClawLeft() {
        clawLeft.setPosition(clawLeftClosePosition);
        telemetry.addLine("Closed left claw");
        telemetry.update();
    }
    public void openClawRight() {
        clawRight.setPosition(clawRightOpenPosition);
        telemetry.addLine("Opened right claw");
        telemetry.update();
    }
    public void closeClawRight() {
        clawRight.setPosition(clawRightClosePosition);
        telemetry.addLine("Closed right claw");
        telemetry.update();
    }

    public void moveWristToInit(double power){
        wristMotor.setTargetPosition(wristInitialPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(power);
        telemetry.addData("Moved Wrist to Initial Position at Power:", power);
        telemetry.update();
    }
    public void moveWristToGround(double power){
        wristMotor.setTargetPosition(wristGroundPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(power);
        telemetry.addData("Moved Wrist to Ground Position at Power:", power);
        telemetry.update();
    }
    public void moveWristToDeposit(double power){
        wristMotor.setTargetPosition(wristDepositPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(power);
        telemetry.addData("Moved Wrist to Deposit Position at Power:", power);
        telemetry.update();
    }

    public void moveArmUp(double power){
        armMotor.setTargetPosition(armUpPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
        telemetry.addData("Moved Arm Up at Power:", power);
        telemetry.update();
    }

    public void moveArmDown(double power){
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
        telemetry.addData("Moved Arm Down at Power:", power);
        telemetry.update();
    }
    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        //Defining a rectangle example: Rect exampleRect = new Rect ( x starting location, y starting location, width, height)
        Rect centerRect = new Rect (705, 535, 165, 170);
        Rect leftRect = new Rect (60,550,180, 170);
        // creating a variable to store the color of the rectangle
        Scalar rectColor = new Scalar (0.0, 0.0, 255.0);

        Mat centerCrop; //empty matrix
        Mat leftCrop; //empty matrix
        double centerAvg; //avg for rgb (it determines what color)
        double leftAvg; //avg for rgb (it determines what color)


        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            Imgproc.rectangle(input, centerRect, rectColor, 3);
            Imgproc.rectangle(input, leftRect, rectColor, 3);

            //grabs the pixels ONLY in the sub matrix (centerRect or rightRect)
            centerCrop = input.submat(centerRect);
            leftCrop = input.submat(leftRect);

            //average RGB of the sub matrix
            Scalar centerRGBavg = Core.mean(centerCrop);
            Scalar leftRGBavg = Core.mean(leftCrop);

            //algorithm for detecting red
            centerAvg = 2*centerRGBavg.val[0] - centerRGBavg.val[1] - centerRGBavg.val[2];
            leftAvg = 2*leftRGBavg.val[0] - leftRGBavg.val[1] - leftRGBavg.val[2];

            // 0 = red, 1 = green, 2 = blue

            // the value at which we say, neither pixel region is red (must be on the right)
            double threshold = 60.0;

            if(centerAvg > leftAvg && centerAvg >= threshold){
                telemetry.addLine("Prop Location: Center");
                pixelLocation = CENTER;
            }
            else if (leftAvg > centerAvg && leftAvg >= threshold){
                telemetry.addLine("Prop Location: Left");
                pixelLocation = LEFT;
            }
            else{
                telemetry.addLine("Prop Location: Right");
                pixelLocation = RIGHT;
            }

            telemetry.addLine("Center: " + centerAvg);
            telemetry.addLine("Left: " + leftAvg);



            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}