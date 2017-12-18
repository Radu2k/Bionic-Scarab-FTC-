package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.sql.Time;
/**
 * Created by dave on 12/16/17.
 */

public class controls extends LinearOpMode{
    //dclaring motors
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor extendDrive = null;
    private DcMotor upDrive = null;
        //declaring tunning variables
    private double upStep=0.1; //how fast to lift the cube
    private double leftPower;
    private double rightPower;
    private double powerRatio=99.0; //acceleration value the closer to 100 the faster the acceleration
    private ElapsedTime timeheigh = new ElapsedTime();
    private ElapsedTime timeextend = new ElapsedTime();

    //main navigation function takes in drive as acceleration forward or backward and turn witch controls steering
    public void navigate(double drive,double turn){
        leftPower = (powerRatio*Range.clip(drive + turn, -1.0, 1.0)+(100.0-powerRatio)*leftPower)/100.0 ;
        rightPower = (powerRatio*Range.clip(drive - turn, -1.0, 1.0)+(100.0-powerRatio)*rightPower)/100.0;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    //
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

    //main setup phase required to set up motors
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        upDrive = hardwareMap.get(DcMotor.class, "up_drive");
        extendDrive= hardwareMap.get(DcMotor.class, "extend_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        upDrive.setDirection(DcMotor.Direction.FORWARD);
        extendDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    controls(){
        runOpMode();
    }
}
