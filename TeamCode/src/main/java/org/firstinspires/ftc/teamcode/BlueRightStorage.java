package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.basket;
import static org.firstinspires.ftc.teamcode.Robot.basketdefault;
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


@Autonomous(name = "<->BLUE Right:Storage", preselectTeleOp = "teleopV2")
public class BlueRightStorage extends LinearOpMode{
    OpenCvCamera webcam;
    enum RobotPath {
        BARRIER,
        GAP
    }
    RobotPath Path;

    final int START_X = -36;
    final int START_Y = 64;
    int level = 0;
    int height = 0;
    double basket_value = 0;
    double alignDistance = 0;


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
        basket.setPosition(basketdefault);
        Path = RobotPath.BARRIER;
        ////

        telemetry.addData("location: ", pipeline.getSide());
        telemetry.update();
        switch(pipeline.getSide()) {
            case LEFT_SIDE:
                level = 1;
                height = 0;
                basket_value = 0.93;
                break;
            case MIDDLE_SIDE:
                level = 2;
                height = 1350;
                basket_value = 0.95;
                break;
            case RIGHT_SIDE:
                level = 3;
                height = 2050;
                basket_value = 0.93;

        }
        Trajectory inchForward = drive.trajectoryBuilder(startPose) //moves bot forward from start and turns
                .strafeTo(new Vector2d(-36, 55))
                .build();

        Trajectory toCarousel = drive.trajectoryBuilder(inchForward.end()) //turn to carousel
                .lineToLinearHeading(new Pose2d(-16, 63.5,Math.toRadians(-55)))//to -90
                .build();


        Trajectory toTurn = drive.trajectoryBuilder(toCarousel.end().plus(new Pose2d(0,0,Math.toRadians(-35))), true) //To turn next to shipping hub
                .forward(29)
                .build();

        Trajectory toHub = drive.trajectoryBuilder(toTurn.end().plus(new Pose2d(0,0,Math.toRadians(-180))), true) //To turn next to shipping hub
                .strafeLeft(30)
                .build();
        Trajectory park = drive.trajectoryBuilder(toHub.end()) //To turn next to shipping hub
                .strafeRight(45)
                .build();
        Trajectory fullPark = drive.trajectoryBuilder(park.end()) //To turn next to shipping hub
                .forward(13)
                .build();







        //drive sequence code
        drive.followTrajectory(inchForward);

        drive.followTrajectory(toCarousel);

        tablemotor.setPower(0.5);
        sleep(2000);
        tablemotor.setPower(0);

        drive.followTrajectory(toTurn);
        drive.turn(Math.toRadians(147));
        drive.followTrajectory(toHub);



        Trajectory toShippingHub2Short = drive.trajectoryBuilder(toTurn.end())//Bottom
                .strafeLeft(9)
                .build();
        Trajectory toShippingHub2Middle = drive.trajectoryBuilder(toTurn.end())//Middle
                .strafeLeft(9.5)
                .build();
        Trajectory toShippingHub2Long = drive.trajectoryBuilder(toTurn.end())//Top
                .strafeLeft(11)
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
        basket.setPosition(Robot.basketdefault);

        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.6);

        drive.followTrajectory(park);
        drive.followTrajectory(fullPark);
        /*
        Trajectory align;
        Trajectory sprint;
        if(Path == RobotPath.GAP) {
            alignDistance = 35;
        } else {
            alignDistance = 3.5;
        }

        if(level == 1 || level == 2) {
            align = drive.trajectoryBuilder(toShippingHub2Short.end())
                    .strafeRight(alignDistance)
                    .build();
            sprint = drive.trajectoryBuilder(align.end())
                    .forward(73.3)
                    .build();
            drive.followTrajectory(align);
            drive.followTrajectory(sprint);
        } else if(level == 3) {
            align = drive.trajectoryBuilder(toShippingHub2Long.end())
                    .strafeRight(alignDistance)
                    .build();
            sprint = drive.trajectoryBuilder(align.end())
                    .forward(73.3)
                    .build();
            drive.followTrajectory(align);
            drive.followTrajectory(sprint);
        }
        */
        if (isStopRequested()) return;
        sleep(2000);


    }
}