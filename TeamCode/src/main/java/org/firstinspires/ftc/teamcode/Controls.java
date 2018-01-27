package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import java.sql.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Thread.holdsLock;
import static java.lang.Thread.sleep;


public class Controls {
    //dclaring motors
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor extendDrive = null;
    public DcMotor upDrive = null;
    ModernRoboticsI2cGyro Gyro;

    Servo grab_cube_left;
    Servo grab_cube_right;

    //declaring tunning variables
    private double upStep=0.5;//how fast to lift the cube
    private double leftPower;
    private int offset=1;
    private double cmPerRotation=13.33;
    private double degreesPerRotation=0.3141592653589793;
    private double rightPower;
    private double powerRatio=99.0;//acceleration value the closer to 100 the faster the acceleration

    private boolean grab_cub_check = true;

    public ElapsedTime timeextend = new ElapsedTime();
    public ElapsedTime timegrab = new ElapsedTime();

    Controls(){
        
        this.leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        this.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        telemetry.addData("set up drive engines","");

        this.upDrive = hardwareMap.get(DcMotor.class, "up_drive");
        this.extendDrive = hardwareMap.get(DcMotor.class, "extend_drive");
        telemetry.addData("set up lifter and extender engines ","");

        this.grab_cube_left = hardwareMap.get(Servo.class,"grab_cube_left");
        this.grab_cube_right = hardwareMap.get(Servo.class,"grab_cube_right");
        telemetry.addData("set up grab servos","");

        this.leftDrive.setDirection(DcMotor.Direction.FORWARD);
        this.rightDrive.setDirection(DcMotor.Direction.REVERSE);
        this.upDrive.setDirection(DcMotor.Direction.FORWARD);
        this.extendDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    
    //main navigation function takes in drive as acceleration forward or backward and turn witch Controls steering

    public void navigate(double drive,double turn){
        leftPower = (powerRatio*Range.clip(drive + turn, -1.0, 1.0)+(100.0-powerRatio)*leftPower)/100.0 ;
        rightPower = (powerRatio*Range.clip(drive - turn, -1.0, 1.0)+(100.0-powerRatio)*rightPower)/100.0;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    public void lifter_up(){
        upDrive.setPower(upStep);
    }

    public void lifter_down(){
        upDrive.setPower(-upStep);
    }

    public void stopmotors(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void moveDistance(int leftStep, int rightStep, double leftPower, double rightPower, double distance) {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setTargetPosition(leftStep);
        rightDrive.setTargetPosition(rightStep);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        while(leftDrive.isBusy() && rightDrive.isBusy()){
        }
        stopmotors();
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void forewordWithDistance(double power, int distance){
        double steps = distance * cmPerRotation;
        int step = (int) steps;
        moveDistance(step, step, power, power, distance);
    }

    public void rotateLeftDegrees(double power, int degrees){
         /* sidenote:
            In implementatia originala era:
            > leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            in loc de versiunea folosita de restul functiilor:
            > leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Intentionat sau bug?
         */
        double steps = degrees * degreesPerRotation * cmPerRotation;
        int step = (int) steps*offset;
        moveDistance(-step, step, -power, power, distance);
    }

    public void rotateRightDegrees(double power ,int degrees){
        // originalul nu folosea run encoded pt right motor, bug?
        double steps = degrees * degreesPerRotation * cmPerRotation;
        int step = (int) steps * offset;
        moveDistance(step, -step, power, -power, distance);
    }

    public void backwardWithDistance(double power,int distance){
        double steps = distance*cmPerRotation;
        int step = (int) steps;
        moveDistance(-step, -step, -power, -power, distance);
    }

    public void grabfirst(){
        grab_cube_right.setPosition(0.7 );
        grab_cube_left.setPosition(0.2 );

    }

    public void grab(){
        if(timegrab.seconds()>0.3) {
            if (grab_cub_check == true) {
                grab_cube_right.setPosition(0.6);

                grab_cube_left.setPosition(0.3);
                grab_cub_check = false;
                timegrab.reset();
                timegrab.startTime();

            } else {

                grab_cube_right.setPosition( 0.9);
                grab_cube_left.setPosition(-0.1);

                grab_cub_check = true;
                timegrab.reset();
                timegrab.startTime();
            }
        }
    }

    public void lifter_stop()
    {
        upDrive.setPower(0.0);
    }

    public void stop_extend_relic(){
        extendDrive.setPower(0);
        timeextend.reset();
        timeextend.startTime();
    }

    private void turnByGyro(double leftPower, double rightPower, double degrees){
        Gyro.resetZAxisIntegrator();
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        while( degrees > Gyro.getIntegratedZValue()){

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void turnLeftByGyro(double power ,double degrees){
        turnByGyro(-power, power, degrees);
    }

    public void turnRightByGyro(double power ,double degrees){
        turnByGyro(power, -power, -degrees);
    }

    public void moveByTime(double power,int time){
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setPower(power);
        leftDrive.setPower(power);
        SystemClock.sleep(time);
        rightDrive.setPower(0);
        leftDrive.setPower(0);
    }

}
