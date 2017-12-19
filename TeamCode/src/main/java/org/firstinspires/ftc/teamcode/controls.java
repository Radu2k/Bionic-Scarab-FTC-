package org.firstinspires.ftc.teamcode;

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

    //declaring tunning variables
    private double upStep=0.1;//how fast to lift the cube
    private double leftPower;
    private double rightPower;
    private double powerRatio=99.0;//acceleration value the closer to 100 the faster the acceleration

    private ElapsedTime timeheigh = new ElapsedTime();
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

    public void checktimecub(Servo servo_power)
    {
        if((timegrab.seconds()>2 && servo_power.getPosition()!=0) )
        {
            servo_power.setPosition(0.0);
        }
    }


    public void lifter_up(){
        upDrive.setPower(upStep);
        if(upDrive.getPower()!=0)
        {
            timeheigh.reset();
            timeheigh.startTime();

        }
    }

    public void checktime()
    {
        if((timeheigh.seconds()>4 && upDrive.getPower()!=0)||(timeheigh.seconds()<-4 && upDrive.getPower()!=0) )
        {
            upDrive.setPower(0.0);
        }
    }

    public void lifter_down(){
        upDrive.setPower(-upStep);
        if(upDrive.getPower()!=0)
        {
            timeheigh.reset();
            timeheigh.startTime();

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
