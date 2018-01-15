package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.sql.Time;


public class controls {
    //dclaring motors
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor extendDrive = null;
    public DcMotor upDrive = null;

    Servo grab_cube_left;
    Servo grab_cube_right;


    //declaring tunning variables
    private double upStep=0.5;//how fast to lift the cube
    private double leftPower;
    private double rightPower;
    private double powerRatio=99.0;//acceleration value the closer to 100 the faster the acceleration

    private boolean grab_cub_check=true;

    private ElapsedTime timeextend = new ElapsedTime();
    private ElapsedTime timegrab = new ElapsedTime();

    //main navigation function takes in drive as acceleration forward or backward and turn witch controls steering

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


    public void grabfirst(){

        grab_cube_right.setPosition(0.7 );
        grab_cube_left.setPosition(0.2 );

    }

    public void grab(){
        if(timegrab.seconds()>0.3
                ) {
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

    public void extend_relic(){

        extendDrive.setPower(1);
        timeextend.reset();
        timeextend.startTime();

    }

    public void retract_relic(){

        extendDrive.setPower(-1);
        timeextend.reset();
        timeextend.startTime();

    }

    public void checktimeextend(){
        if(timeextend.seconds()>4)
        {
            extendDrive.setPower(0);
            timeextend.reset();
        }

    }


}
