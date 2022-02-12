package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.RunIntake;
import static org.firstinspires.ftc.teamcode.Robot.basket;
import static org.firstinspires.ftc.teamcode.Robot.basketdefault;
import static org.firstinspires.ftc.teamcode.Robot.cap;
import static org.firstinspires.ftc.teamcode.Robot.capdefault;
import static org.firstinspires.ftc.teamcode.Robot.initAccessories;
import static org.firstinspires.ftc.teamcode.Robot.initMotors;
import static org.firstinspires.ftc.teamcode.Robot.slide;
import static org.firstinspires.ftc.teamcode.Robot.tablemotor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BlueLeft", preselectTeleOp = "teleopV2")
public class BLUELeft extends LinearOpMode{
    OpenCvCamera webcam;
    final int START_X = 36;
    final int START_Y = 64;
    int level = 0;
    int height = 0;
    double basket_value;

    @Override
    public void runOpMode() throws InterruptedException {
        ////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        initMotors(this);
        initAccessories(this);

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(-90)); //init starting position
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
                telemetry.update();
            }
        });


////////Program start////////////////////////////////////////////////////////////////////////

        waitForStart();
        ////Move on start/init
        basket.setPosition(basketdefault);
        cap.setPosition(capdefault);
        ////

        switch(pipeline.getSide()) {
            case LEFT_SIDE:
                level = 1;
                height = 0;
                basket_value = 0.95;
                break;
            case MIDDLE_SIDE:
                level = 2;
                height = 1350;
                basket_value = 0.95;
                break;
            case RIGHT_SIDE:
                level = 3;
                height = 2050;
                basket_value = 0.96;

        }

        double added = 0;
        if (level == 2) added = 1;
        else if (level == 3) added = 5;

        Trajectory deliverPreload = drive.trajectoryBuilder(startPose) //moves bot forward from start and turns
                .lineToLinearHeading(new Pose2d(14, 86 + added, Math.toRadians(-20)))
                .build();

        Trajectory shippingToWall = drive.trajectoryBuilder(deliverPreload.end()) //moves bot from hub to wall
                .lineToLinearHeading(new Pose2d(30, 36, Math.toRadians(114)))
                .build();

        Trajectory toWarehouse = drive.trajectoryBuilder(shippingToWall.end()) //moves bot from wall in to warehouse
                .back(42)
                .build();

        Trajectory warehouseToWall = drive.trajectoryBuilder(toWarehouse.end()) //moves bot from warehouse to wall
                .forward(68)
                .build();

        Trajectory toHub = drive.trajectoryBuilder(warehouseToWall.end()) //moves bot from wall to hub
                .lineToLinearHeading(new Pose2d(27, 91, Math.toRadians(-25)))
                .build();

        // Deliver preload block
        drive.followTrajectory(deliverPreload);

        slide.setTargetPosition(height);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.6);
        while(slide.isBusy()){}


        basket.setPosition(basket_value);
        sleep(2000);
        basket.setPosition(Robot.basketdefault);

        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.6);

        height = 2050;
        basket_value = 0.93;

        for(int i = 0; i < 1; i++) {
            //changes how far the robot goes foward to account for ball and block pushing
            Trajectory toWarehouseLong = drive.trajectoryBuilder(shippingToWall.end())
                    .back(22 + (i * 5))
                    .build();
            //changes how far the robot goes back to account for ball and block pushing
            Trajectory warehouseToWallLong = drive.trajectoryBuilder(toWarehouse.end())
                    .forward(40 + (i * 5))
                    .build();

            //re-aligns the bot by pushing against the wall
            Trajectory align = drive.trajectoryBuilder(toWarehouseLong.end())
                    .strafeLeft(9)
                    .build();

            // go to the wall from the shipping hub and begin intaking
            drive.followTrajectory(shippingToWall);

            RunIntake(-.90);


            drive.followTrajectory(toWarehouseLong);

            sleep(1000);
            drive.followTrajectory(align);
            RunIntake(.90);

            drive.followTrajectory(warehouseToWallLong);
            slide.setTargetPosition(height);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.4);


            RunIntake(0);
            drive.followTrajectory(toHub);
            while(slide.isBusy()){}

            basket.setPosition(basket_value);
            sleep(2500);
            basket.setPosition(Robot.basketdefault);

            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.6);
        }

        Trajectory toPark = drive.trajectoryBuilder(toHub.end())
                .strafeRight(32)
                .build();
        drive.followTrajectory(toPark);
        Trajectory Park = drive.trajectoryBuilder(toPark.end())
                .forward(50)
                .build();
        drive.followTrajectory(Park);

        if (isStopRequested()) return;
        sleep(2000);


    }
}
