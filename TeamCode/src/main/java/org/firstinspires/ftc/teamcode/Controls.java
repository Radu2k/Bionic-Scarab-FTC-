package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import java.sql.Time;


public class Controls {
    //dclaring motors
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor upDrive = null;
    public ModernRoboticsI2cGyro   gyro    = null;

    Servo grab_cube_left;
    Servo grab_cube_right;



    //declaring tunning variables
    private double upStep=0.5;//how fast to lift the cube
    private double leftPower;
    private double rightPower;
    public double powerRatio=75;//acceleration value the closer to 100 the faster the acceleration

    private boolean grab_cub_check=true;


    public ElapsedTime timegrab = new ElapsedTime();
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

    public void grab(){
        if(timegrab.seconds()>0.3) {
            if (grab_cub_check == true) {
                grab_cube_right.setPosition(0.6);
                grab_cube_left.setPosition(0.4);
                grab_cub_check = false;
                timegrab.reset();
                //timegrab.startTime();

            } else {

                grab_cube_right.setPosition(0.4);
                grab_cube_left.setPosition(0.7);
                grab_cub_check = true;
                timegrab.reset();
                //timegrab.startTime();
            }
        }
    }

    public void lifter_stop(){

        upDrive.setPower(0.0);
    }

    public final void sleep(long milliseconds) {

        SystemClock.sleep(milliseconds);
    }


}
