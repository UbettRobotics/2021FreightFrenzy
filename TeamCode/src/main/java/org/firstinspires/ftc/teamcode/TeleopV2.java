package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Robot.*;
import static org.firstinspires.ftc.teamcode.Robot.RunIntake;
import static org.firstinspires.ftc.teamcode.Robot.RunSlide;
import static org.firstinspires.ftc.teamcode.Robot.basket;
import static org.firstinspires.ftc.teamcode.Robot.slide;
import static org.firstinspires.ftc.teamcode.Robot.tablemotor;

@TeleOp(name = "teleopV2")
public class TeleopV2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);
        initAccessories(this);
        initIMU(this);

        waitForStart();

        int CurrentLevel = 0;
        final int LEVEL0 = 0;
        final int LEVEL1 = 800; //Unused
        final int LEVEL2 = 1350;
        final int LEVEL3 = 2050;
        Boolean tipped = false;

        while(opModeIsActive()) {
            boolean LBumper1 = gamepad1.left_bumper;
            boolean RBumper1 = gamepad1.right_bumper;

            double LStickY = gamepad1.left_stick_y;
            double LStickX  = gamepad1.left_stick_x; // was "-gamepad..." by connor
            double RStickY = -gamepad1.right_stick_y;
            double RStickX = -gamepad1.right_stick_x;

            double LTrigger1 = gamepad1.left_trigger; // was "-g..."
            double RTrigger1 = gamepad1.right_trigger; // was "-g..."

            boolean a1 = gamepad1.a;
            boolean b1 = gamepad1.b;
            boolean x1 = gamepad1.x;
            boolean y1 = gamepad1.y;

            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            boolean x2 = gamepad2.x;
            boolean y2 = gamepad2.y;

            double LTrigger2 = gamepad2.left_trigger;
            double RTrigger2 = gamepad2.right_trigger;
            boolean LBumper2 = gamepad2.left_bumper;
            boolean RBumper2 = gamepad2.right_bumper;

            double RStickY2 = -gamepad2.right_stick_y;
            double RStickX2 = gamepad2.right_stick_x;
            double LStickY2 = -gamepad2.left_stick_y;
            double LStickX2 = gamepad2.left_stick_x;

            boolean dpadUp1 = gamepad1.dpad_up;
            boolean dpadDown1 = gamepad1.dpad_down;
            boolean dpadRight1 = gamepad1.dpad_right;
            boolean dpadLeft1 = gamepad1.dpad_left;

            boolean dpadUP2 = gamepad2.dpad_up;
            boolean dpadDOWN2 =gamepad2.dpad_down;
            boolean dpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;

            //diagonal driving
            if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
                if (Math.abs(LStickX) < .05 && Math.abs(RStickX) < .05) {
                    SetPower(LStickY, LStickY, LStickY, LStickY);
                }
                else if (Math.abs(LStickY) < .05 && Math.abs(RStickX) < .05) {
                    SetPower(LStickX, -LStickX, -LStickX, LStickX);//+--+
                }
                else {
                    imu.getAngularOrientation();
                    double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                    double rightX = -gamepad1.right_stick_x;

                    double v1 = r * Math.cos(robotAngle) + rightX; //lf
                    double v2 = r * Math.sin(robotAngle) - rightX; //rf
                    double v3 = r * Math.sin(robotAngle) + rightX; //lb
                    double v4 = r * Math.cos(robotAngle) - rightX; //rb

                    SetPower(v1, v2, v3, v4);
                }
            }

            //Trigger Turning
            else if (Math.abs(LTrigger1) > 0) {
                SetPower(-.25 * LTrigger1, .25 * LTrigger1, -.25 * LTrigger1, .25 * LTrigger1); //.25
            }
            else if (Math.abs(RTrigger1) > 0) {
                SetPower(.25 * RTrigger1, -.25 * RTrigger1, .25 * RTrigger1, -.25 * RTrigger1); //.25
            }

            else if (LBumper1) {
                SetPower(-.5 * 1, .5 * 1, -.5 * 1, .5 * 1);
            }
            else if (RBumper1) {
                SetPower(.5 * 1, -.5 * 1, .5 * 1, -.5 * 1);
            }
            else {
                SetPower(0,0,0,0);
            }


            //driving
            if (Math.abs(LStickY) > 0) {
                SetPower(LStickY, LStickY, LStickY, LStickY);
            }
            else if(Math.abs(RStickX) > 0) {
                SetPower(RStickX, -RStickX, RStickX, -RStickX);
            }
            //turning
            else if (Math.abs(LTrigger1) > 0) {
                SetPower(-LTrigger1, LTrigger1, LTrigger1, -LTrigger1);
            }
            else if (Math.abs(RTrigger1) > 0) {
                SetPower(RTrigger1, -RTrigger1, -RTrigger1, RTrigger1);
            }
            //d pad fine tuned driving
            if(dpadUp1){
                SetPower(-.5, -.5, -.5, -.5); //0.5
            }
            else if(dpadRight1){
                SetPower(.5, -.5, -.5, .5); //0.5
            }
            else if(dpadLeft1){
                SetPower(-.5, .5, .5, -.5);
            }
            else if(dpadDown1){
                SetPower(.5, .5, .5, .5);
            }


            //carousel
            if (RBumper2){
                tablemotor.setPower(1);
            }
            else if(LBumper2) {
                tablemotor.setPower(-1);
            }
            else{
                tablemotor.setPower(0);
            }

            //dump basket
            if (y2) {
                basket.setPosition(.95);
                tipped = true;
            } else if (x2) { //shared
                basket.setPosition(.99);
                tipped = true;
            } else if (b2){
                basket.setPosition(.80);
            }else {
                basket.setPosition(basketdefault);
                if(CurrentLevel == LEVEL0 ){
                    tipped = false;
                }
                else if(!slide.isBusy() && tipped == true){
                    RunSlide(LEVEL1,1, dpadUP2, dpadDOWN2);
                    CurrentLevel = LEVEL0;
                    tipped = false;
                }
            }








            // intake motor
            if (LTrigger2 > .05){
                RunIntake(.90);
            }
            else if(RTrigger2 > .05) {
                RunIntake(-.90);
            }
            else{
                RunIntake(0.0);
            }

            //linear slide
            if(dpadUP2){
                CurrentLevel = LEVEL3;
            }
            else if(dpadLeft2){
                CurrentLevel = LEVEL2;
            }
            else if(dpadDOWN2) {
                CurrentLevel = LEVEL0;
            }else if(a1 && a2 && b1){
                CurrentLevel = 5;
            }
            //update linear slide position
            RunSlide(CurrentLevel, 1, dpadUP2, dpadDOWN2);


            if(a1 && a2 && b1){
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if(slide.getCurrentPosition() == 0 && limit.isPressed()){
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }




                //telemetry/////////////////////////////////////////////////////////////////////////////
            telemetry.addData("ECV", slide.getCurrentPosition());
            telemetry.addData("Target",CurrentLevel);
            telemetry.addData("IMU", imu.getAngularOrientation());
            telemetry.addData("X", imu.getAngularVelocity().xRotationRate);
            telemetry.addData("Y", imu.getAngularVelocity().yRotationRate);
            telemetry.addData("Z", imu.getAngularVelocity().zRotationRate);
            telemetry.addData("D",distance.getDistance(cm));
            telemetry.addData("Touch Sensor", limit.isPressed());
            telemetry.update();



        }


    }
}
