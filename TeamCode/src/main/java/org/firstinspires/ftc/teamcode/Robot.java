package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {

    public Robot() {}

    final static int TICKS_PER_INCH = 33;


    //public final static double BLOCKER_OPEN = 0.25;
    public static double basketdefault = 0.51;
    final static double DEG90 = 22;//90 degree turn distance ()

    static DcMotor rightfront;
    static DcMotor leftfront;
    static DcMotor leftback;
    static DcMotor rightback;

    static DcMotor frontintake;
    static DcMotor tablemotor;

    static DcMotor slide;

    static Servo basket;
    static BNO055IMU imu;
    static Orientation lastAngles = new Orientation();
    static DistanceSensor distance;
    static TouchSensor limit;
    static double globalAngle, correction;
    static DistanceUnit cm = DistanceUnit.CM;



    public static void initMotors(OpMode opMode) {
        rightfront = opMode.hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = opMode.hardwareMap.get(DcMotor.class, "leftfront");
        leftback = opMode.hardwareMap.get(DcMotor.class, "leftback");
        rightback = opMode.hardwareMap.get(DcMotor.class, "rightback");

        tablemotor = opMode.hardwareMap.get(DcMotor.class, "tableMotor");
        frontintake = opMode.hardwareMap.get(DcMotor.class, "frontintake");
        slide = opMode.hardwareMap.get(DcMotor.class,"slide");
        basket = opMode.hardwareMap.servo.get("basket");
        distance = opMode.hardwareMap.get(DistanceSensor.class, "distance");
        limit = opMode.hardwareMap.get(TouchSensor.class, "limit");


        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        tablemotor.setDirection(DcMotor.Direction.FORWARD);

        frontintake.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors();
    }

    public static void initAccessories(OpMode opMode){
    }

    public static void resetMotors() {
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontintake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tablemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tablemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void forward(double dist, double speed) {
        move(speed, dist, dist, dist, dist);
    }
    public static void back(double dist, double speed) {
        move(speed, -dist, -dist, -dist, -dist);
    }
    public static void counter(double dist, double speed) {
        move(speed, -dist, -dist, dist, dist);
    }
    public static void clock(double dist, double speed){ move(speed, dist, dist, -dist, -dist); }
    public static void right(double dist, double speed) { move(speed, -dist, dist, dist, -dist); }
    public static void left(double dist, double speed) { move(speed, dist, -dist, -dist, dist); }


    public static void resetEncoders(){
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tablemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static double[] encoderValues(){
        double[] encoderArray = {rightfront.getCurrentPosition(), rightback.getCurrentPosition(), leftfront.getCurrentPosition(), leftback.getCurrentPosition(), frontintake.getCurrentPosition()};
        return encoderArray;
    }

    public static void move(double speed, double distRF, double distRB, double distLF, double distLB) {
        resetEncoders();

        int posRF = (int)(distRF * TICKS_PER_INCH);
        int posRB = (int)(distRB * TICKS_PER_INCH);
        int posLF = (int)(distLF * TICKS_PER_INCH);
        int posLB = (int)(distLB * TICKS_PER_INCH);


        rightfront.setTargetPosition(posRF);
        rightback.setTargetPosition(posRB);
        leftfront.setTargetPosition(posLF);
        leftback.setTargetPosition(posLB);

        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightfront.setPower(speed);
        rightback.setPower(speed);
        leftfront.setPower(speed);
        leftback.setPower(speed);

        while(rightfront.isBusy() && rightback.isBusy() && leftfront.isBusy() && leftback.isBusy()) {

        }

        rightfront.setPower(0);
        rightback.setPower(0);
        leftfront.setPower(0);
        leftback.setPower(0);
    }

    public static void SetPower(double LFPower, double RFPower, double LBPower, double RBPower) {
        //leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftfront.setPower(LFPower);
        leftback.setPower(LBPower);
        rightfront.setPower(RFPower);
        rightback.setPower(RBPower);


    }

    //intake
    public static void RunIntake(double Frontintake){
        frontintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontintake.setPower(Frontintake);
    }
    public static void RunSlide(double level, double power, Boolean up, Boolean down){
        //level can be double 0-10
        slide.setTargetPosition((int)(level));
        if(slide.getMode().compareTo(DcMotor.RunMode.RUN_WITHOUT_ENCODER)==0){
            // determine basket going up, down, or nothing
            int multiplier = 0;
            if (up == true) multiplier = 1;
            else if (down == true) multiplier = -1;
            slide.setPower(.35 * multiplier);
        }
        else {
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(power);
        }
    }
    public static void RunSlide(double level, double power){
        //level can be double 0-10
        slide.setTargetPosition((int)(level));
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
    }
    public static void initIMU(OpMode opMode){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        //This line is only necessary if the the Control Hub is mounted vertically (as done this year)
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        opMode.telemetry.addData("Mode: ", "imu calibrating");
        opMode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
        }
        opMode.telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
        opMode.telemetry.update();
        resetAngle();
    }

    public static double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public static void moveToAngle(double angle, double power){
        double  leftPower, rightPower;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (angle > getAngle()) {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (angle < getAngle()) {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        SetPower(leftPower, rightPower, leftPower, rightPower);



        // rotate until turn is completed.
        if(getAngle() > angle){
            while (getAngle() > angle) {}
        } else{
            while (getAngle() < angle) {}
        }

       /* if (angle > getAngle()) {
            // On right turn we have to get off zero first.

            while (getAngle() > angle) {}
        }
        else    // left turn.
            while (getAngle() < angle) {}
        */
        SetPower(0,0,0,0);
    }
    private static void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
}
