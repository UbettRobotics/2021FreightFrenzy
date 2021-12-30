package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.basket;
import static org.firstinspires.ftc.teamcode.Robot.initAccessories;
import static org.firstinspires.ftc.teamcode.Robot.initMotors;
import static org.firstinspires.ftc.teamcode.Robot.slide;
import static org.firstinspires.ftc.teamcode.Robot.tablemotor;

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


@Autonomous(name = "<->BLUE Right:Barrier", preselectTeleOp = "teleopV2")
public class BLUERightBarrier extends LinearOpMode{
    OpenCvCamera webcam;
    final int START_X = -36;
    final int START_Y = 64;
    int level = 0;
    int height = 0;
    double basket_value = 0;

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
                telemetry.addData("Please restart the program", 0);
                telemetry.update();
            }
        });

////////Program start////////////////////////////////////////////////////////////////////////

        waitForStart();
        ////Move on start/init
        basket.setPosition(0.5);
        ////

        telemetry.addData("location: ", pipeline.getSide());
        telemetry.update();
        switch(pipeline.getSide()) {
            case LEFT_SIDE:
                level = 1;
                height = 0;
                basket_value = 0.94;
                break;
            case MIDDLE_SIDE:
                level = 2;
                height = -1350;
                basket_value = 0.95;
                break;
            case RIGHT_SIDE:
                level = 3;
                height = -2050;
                basket_value = 0.93;

        }
        Trajectory inchForward = drive.trajectoryBuilder(startPose) //moves bot forward from start and turns
                .strafeTo(new Vector2d(-36, 55))
                .build();

        Trajectory toCarousel = drive.trajectoryBuilder(inchForward.end()) //moves bot forward from start and turns
                .lineToLinearHeading(new Pose2d(-10, 62.5,Math.toRadians(-55)))//to -90
                .build();


        Trajectory toTurn = drive.trajectoryBuilder(toCarousel.end().plus(new Pose2d(0,0,Math.toRadians(-35))), true) //To turn next to shipping hub
                .lineToLinearHeading(new Pose2d(-50, 70,Math.toRadians(-179)))//to -180
                .build();




        //drive sequence code
        drive.followTrajectory(inchForward);

        drive.followTrajectory(toCarousel);

        tablemotor.setPower(0.5);
        sleep(2000);
        tablemotor.setPower(0);

        drive.followTrajectory(toTurn);
        stop();


        Trajectory toShippingHub2Short = drive.trajectoryBuilder(toTurn.end())//Bottom
                .lineTo(new Vector2d(-70, 105))
                .build();
        Trajectory toShippingHub2Middle = drive.trajectoryBuilder(toTurn.end())//Middle
                .lineTo(new Vector2d(-71, 105))
                .build();
        Trajectory toShippingHub2Long = drive.trajectoryBuilder(toTurn.end())//Top
                .lineTo(new Vector2d(-70, 104))
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

        basket.setPosition(basket_value);
        sleep(3000);
        basket.setPosition(0.5);

        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.6);

        //drive.turn(Math.toRadians(-10));


        Trajectory align;
        Trajectory sprint;
        if(level == 1) {
            align = drive.trajectoryBuilder(toShippingHub2Short.end())
                    .lineTo(new Vector2d(-65,112))
                    .build();
            drive.followTrajectory(align);
            sprint = drive.trajectoryBuilder(align.end()) //Different start points
                    .forward(70)
                    .build();
            drive.followTrajectory(sprint);

        } else if(level == 2) {
            align = drive.trajectoryBuilder(toShippingHub2Middle.end())
                    .lineTo(new Vector2d(-65,112))
                    .build();
            drive.followTrajectory(align);
            sprint = drive.trajectoryBuilder(align.end()) //Different start points
                    .forward(70)
                    .build();
            drive.followTrajectory(sprint);

        } else {
            align = drive.trajectoryBuilder(toShippingHub2Long.end())
                    .lineTo(new Vector2d(-65,112))
                    .build();
            drive.followTrajectory(align);
            sprint = drive.trajectoryBuilder(align.end()) //Different start points
                    .forward(70)
                    .build();
            drive.followTrajectory(sprint);
        }

        if (isStopRequested()) return;
        sleep(2000);


    }
}
