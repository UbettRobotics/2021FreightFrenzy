package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.basket;
import static org.firstinspires.ftc.teamcode.Robot.initAccessories;
import static org.firstinspires.ftc.teamcode.Robot.initMotors;
import static org.firstinspires.ftc.teamcode.Robot.slide;
import static org.firstinspires.ftc.teamcode.Robot.tablemotor;
import static org.firstinspires.ftc.teamcode.Robot.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BLUE Right", preselectTeleOp = "teleopV2")
public class BLUERight extends LinearOpMode{
    OpenCvCamera webcam;
    final int START_X = -36;
    final int START_Y = 64;
    int level = 0;
    int height = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        initMotors(this);
        initAccessories(this);

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(90)); //init starting position
        drive.setPoseEstimate(startPose);

        //////Start Camera Streaming//////
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        /*
        ConeVisionPipeline pipeline = new ConeVisionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", errorCode);
                telemetry.update();
            }
        });
         */

////////Program start////////////////////////////////////////////////////////////////////////

        waitForStart();
        ////Move on start/init
        //basket.setPosition(0.48);
        ////

        telemetry.addData("No Camera", 1);
        Trajectory inchForward = drive.trajectoryBuilder(startPose) //moves bot forward from start and turns
                .lineTo(new Vector2d(-36, 55))
                .build();


        Trajectory toCarousel = drive.trajectoryBuilder(inchForward.end().plus(new Pose2d(-56, 55, Math.toRadians(270))), false) //moves bot forward from start and turns
                .strafeTo(new Vector2d(-56, 55))
                .build();


        //Trajectory toPreDeliveryTurn = drive.trajectoryBuilder(drive.getPoseEstimate(), true) //moves bot forward from start and turns.lineTo(new Vector2d(-36, 48)).build();



        //drive sequence code
        drive.followTrajectory(inchForward);
        drive.turn(Math.toRadians(90));
        drive.turn(Math.toRadians(90));
        drive.setPoseEstimate(new Pose2d(-36, 55, Math.toRadians(270)));

        drive.followTrajectory(toCarousel);
        tablemotor.setPower(0.5);
        sleep(2000);
        tablemotor.setPower(0);

        //drive.setPoseEstimate(new Pose2d(-56, 55, Math.toRadians(270)));

        //drive.followTrajectory(toPreDeliveryTurn);
        //drive.turn(Math.toRadians(90));
        //drive.setPoseEstimate(new Pose2d(-36, 55, Math.toRadians(0)));


        /*
        drive.followTrajectory(inchForward);
        drive.turn(Math.toRadians(30));
        //drive.setPoseEstimate(new Pose2d(-64,46, Math.toRadians(180)));

        Trajectory carouselAdj  ust = drive.trajectoryBuilder(toCarousel.end()) //Different start points
                .lineTo(new Vector2d(-70.5,43))
                .build();
        drive.followTrajectory(carouselAdjust);

        tablemotor.setPower(0.5);
        sleep(2000);
        tablemotor.setPower(0);
        drive.setPoseEstimate(new Pose2d(-70.5,43, Math.toRadians(90)));

        Trajectory toTurn = drive.trajectoryBuilder(carouselAdjust.end())
                .lineTo(new Vector2d(-84.5, 8))
                .build();
        drive.followTrajectory(toTurn);
        drive.turn(Math.toRadians(-90));
        drive.setPoseEstimate(new Pose2d(-84.5,13,Math.toRadians(180)));

        Trajectory toShippingHub2Short = drive.trajectoryBuilder(toTurn.end())//Bottom
                .lineTo(new Vector2d(-70, 0))
                .build();
        Trajectory toShippingHub2Middle = drive.trajectoryBuilder(toTurn.end())//Middle
                .lineTo(new Vector2d(-70, 0))
                .build();
        Trajectory toShippingHub2Long = drive.trajectoryBuilder(toTurn.end())//Top
                .lineTo(new Vector2d(-70, 0))
                .build();

        if(level == 1) {
            drive.followTrajectory(toShippingHub2Short);
        } else if(level == 2) {
            drive.followTrajectory(toShippingHub2Middle);
        } else {
            drive.followTrajectory(toShippingHub2Long);
        }

        //Deliver

        slide.setTargetPosition(height);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.6);
        while(slide.isBusy()){}

        basket.setPosition(0.04);
        sleep(4000);
        basket.setPosition(0.48);

        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.6);
        Trajectory parkStorage;
        Trajectory parkStorage2;
        if(level == 1) {
            parkStorage = drive.trajectoryBuilder(toShippingHub2Short.end()) //Different start points
                    .lineTo(new Vector2d(-95, -5)) // 15
                    .build();
            drive.followTrajectory(parkStorage);
            parkStorage2 = drive.trajectoryBuilder(parkStorage.end()) //Different start points
                    .lineTo(new Vector2d(-105, 26))
                    .build();
            drive.followTrajectory(parkStorage2);
        } else if(level == 2) {
            parkStorage = drive.trajectoryBuilder(toShippingHub2Middle.end()) //Different start points
                    .lineTo(new Vector2d(-95, -5)) // 30
                    .build();
            drive.followTrajectory(parkStorage);
            parkStorage2 = drive.trajectoryBuilder(parkStorage.end()) //Different start points
                    .lineTo(new Vector2d(-105, 26))
                    .build();
            drive.followTrajectory(parkStorage2);
        } else {
            parkStorage = drive.trajectoryBuilder(toShippingHub2Long.end()) //Different start points
                    .lineTo(new Vector2d(-95, -5)) // 25
                    .build();
            drive.followTrajectory(parkStorage);
            parkStorage2 = drive.trajectoryBuilder(parkStorage.end()) //Different start points
                    .lineTo(new Vector2d(-105, 26))
                    .build();
            drive.followTrajectory(parkStorage2);
        }

        if (isStopRequested()) return;
        sleep(2000);
         */

    }
}
