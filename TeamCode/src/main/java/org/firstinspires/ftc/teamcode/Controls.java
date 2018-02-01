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
    static  double CLOSE_ENOUGH_TO_ZERO=3.5;
    private double upStep=0.5;//how fast to lift the cube
    private double leftPower;
    private int offset=2;
    private double cmPerRotation=13.333333333333333;
    private double degreesPerRotation=0.3141592653589793;
    private double rightPower;
    private double powerRatio=99.0;//acceleration value the closer to 100 the faster the acceleration

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    private int ZAccumulated;

    private boolean grab_cub_check=true;
    private boolean ball_arm_check=true;




    public ElapsedTime timeextend = new ElapsedTime();



    public ElapsedTime timegrab = new ElapsedTime();
    public ElapsedTime timeball=new ElapsedTime();

    //main navigation function takes in drive as acceleration forward or backward and turn witch Controls steering

    public void navigate(double drive,double turn){
        leftPower = (powerRatio*Range.clip(drive + turn, -1.0, 1.0)+(100.0-powerRatio)*leftPower)/100.0 ;
        rightPower = (powerRatio*Range.clip(drive - turn, -1.0, 1.0)+(100.0-powerRatio)*rightPower)/100.0;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        //hwMap.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    public void lifter_up(){
        upDrive.setPower(upStep);
    }

    public void lifter_down(){
        upDrive.setPower(-upStep);
    }

    public void forewordWithDistance(double power ,int distance){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double steps = distance*cmPerRotation;
        int step = (int) steps;
        leftDrive.setTargetPosition(step);
        rightDrive.setTargetPosition(step);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }
    

    public void grabfirst(){

        grab_cube_right.setPosition(0.7 );
        grab_cube_left.setPosition(0.2 );

    }

    public void ballArmRetract(){
        if(timegrab.seconds()>0.3) {
            if (ball_arm_check == true) {
                ball_servo.setPosition(0);
                grab_cub_check = false;
                timegrab.reset();
                timegrab.startTime();
            }
            grab_cub_check = true;
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
;

    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void turnLeftByGyro(double power ,double degrees){
        gyro.resetZAxisIntegrator();
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setPower(power);
        leftDrive.setPower(-power);
        while( degrees > Gyro.getIntegratedZValue()){
            if(degrees-Gyro.getIntegratedZValue() <= CLOSE_ENOUGH_TO_ZERO)
                break;

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void turnRightByGyro(double power ,double degrees){
        gyro.resetZAxisIntegrator();
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setPower(-power);
        leftDrive.setPower(power);
        while( -degrees < Gyro.getIntegratedZValue()){
            if(degrees+Gyro.getIntegratedZValue()<=CLOSE_ENOUGH_TO_ZERO)
                break;

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
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
    void grab(){
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

}
