package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import java.sql.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.sun.tools.doclint.Entity.and;


public class Controls {
    //dclaring motors
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor extendDrive = null;
    public DcMotor upDrive = null;
    public ModernRoboticsI2cGyro   gyro    = null;

    Servo grab_cube_left;
    Servo grab_cube_right;
    Servo ball_servo;


    //declaring tunning variables
    static  double CLOSE_ENOUGH_TO_ZERO=24;
    private double upStep=0.5;//how fast to lift the cube
    private double leftPower;
    private double rightPower;
    private double powerRatio=75;//acceleration value the closer to 100 the faster the acceleration



    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.

    private boolean grab_cub_check=true;
    public boolean ball_check=true;


    public ElapsedTime timegrab = new ElapsedTime();
    public ElapsedTime timeball=new ElapsedTime();

    //main navigation function takes in drive as acceleration forward or backward and turn witch Controls steering

    public void navigate(double drive,double turn){
        leftPower = (powerRatio*Range.clip(drive + turn, -1.0, 1.0)+(100.0-powerRatio)*leftPower)/100.0 ;
        rightPower = (powerRatio*Range.clip(drive - turn, -1.0, 1.0)+(100.0-powerRatio)*rightPower)/100.0;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void lifter_up() {

        upDrive.setPower(upStep);
    }

    public void lifter_down(){

        upDrive.setPower(-upStep);
    }

    public void grabfirst(){

        grab_cube_right.setPosition(0.6);
        grab_cube_left.setPosition(0.3);

    }

    public void grab(){
        if(timegrab.seconds()>0.3) {
            if (grab_cub_check == true) {
                grab_cube_right.setPosition(0.3);
                grab_cube_left.setPosition(0.5);
                grab_cub_check = false;
                timegrab.reset();
                timegrab.startTime();

            } else {

                grab_cube_right.setPosition(0.6);
                grab_cube_left.setPosition(0.2);
                grab_cub_check = true;
                timegrab.reset();
                timegrab.startTime();
            }
        }
    }

    public void lifter_stop(){

        upDrive.setPower(0.0);
    }

    public final void sleep(long milliseconds) {

        SystemClock.sleep(milliseconds);
    }

    public void turnLeftByGyro(double power ,double degrees){
        gyro.resetZAxisIntegrator();
        while( degrees - gyro.getIntegratedZValue()>CLOSE_ENOUGH_TO_ZERO){
            rightDrive.setPower(power);
            leftDrive.setPower(-power);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }

    public void turnRightByGyro(double power ,double degrees){
        gyro.resetZAxisIntegrator();
        while( degrees + gyro.getIntegratedZValue()>CLOSE_ENOUGH_TO_ZERO){
            rightDrive.setPower(-power);
            leftDrive.setPower(power);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }

    public void stopBallArm(){
        if(timeball.seconds()>0.3) {
            ball_servo.setPosition(0.5);
            ball_check = false;
            timeball.reset();
            timeball.startTime();}
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

    public void goBallArm(){
        if( ball_check==false) {
            ball_servo.setPosition(0);
            ball_check = true;


        }
        else
        {ball_servo.setPosition(1);
            ball_check = false;
            sleep(1000);
         ball_servo.setPosition(0.5);
                     }


    }


}
