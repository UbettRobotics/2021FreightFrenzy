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

@TeleOp(name = "wheelTest")
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);
        initAccessories(this);
        initIMU(this);

        waitForStart();
        while(opModeIsActive()) {
            boolean LBumper1 = gamepad1.left_bumper;
            boolean RBumper1 = gamepad1.right_bumper;

            double LStickY = gamepad1.left_stick_y;
            double LStickX  = -gamepad1.left_stick_x;
            double RStickY = -gamepad1.right_stick_y;
            double RStickX = -gamepad1.right_stick_x;

            double LTrigger1 = -gamepad1.left_trigger;
            double RTrigger1 = -gamepad1.right_trigger;

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

            /*
            if (dpadUp1) {
                SetPower(.5, 0, 0, 0); //0.5
            } else if (dpadRight1) {
                SetPower(0, .5, 0, 0); //0.5
            } else if (dpadLeft1) {
                SetPower(0, 0, .5, 0);
            } else if (dpadDown1) {
                SetPower(0, 0, 0, .5);
            }
            else {
                SetPower(0, 0, 0, 0);
            }
             */
            boolean Barray[] = {a1,b1,x1,y1, LBumper1,RBumper1,dpadUp1,dpadLeft1,dpadDown1,dpadRight1};
            double Darray[] = {LTrigger1,RTrigger1,RStickX,LStickX,RStickY,LStickY};

            telemetry.addData("Input", Barray[0]);
            telemetry.addData("Input", Barray[1]);
            telemetry.addData("Input", Barray[2]);
            telemetry.addData("Input", Barray[3]);
            telemetry.addData("Input", Barray[4]);
            telemetry.addData("Input", Barray[5]);
            telemetry.addData("Input", Barray[6]);
            telemetry.addData("Input", Barray[7]);
            telemetry.addData("Input", Barray[8]);
            telemetry.addData("Input", Barray[9]);

            telemetry.addData("Input", Darray[0]);
            telemetry.addData("Input", Darray[1]);
            telemetry.addData("Input", Darray[2]);
            telemetry.addData("Input", Darray[3]);
            telemetry.addData("Input", Darray[4]);
            telemetry.addData("Input", Darray[5]);
            telemetry.update();
        }

    }
}