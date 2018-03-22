/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE ODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@Autonomous(name="test", group ="Autonomous")
public class Test extends LinearOpMode {

    private ColorSensor color_sensor;
    private Controls control = new Controls();

    VuforiaLocalizer vuforia;



    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.3;
    static  double     TURN_SPEED_1  = 0.2;


    public void turnAbsolute(int target) {
        int zAccumulated = control.gyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.2;

        while (Math.abs(zAccumulated - target) > 2.7 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                control.leftDrive.setPower(turnSpeed);
                control.rightDrive.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                control.leftDrive.setPower(-turnSpeed);
                control.rightDrive.setPower(turnSpeed);
            }

            zAccumulated = control.gyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }

        control.leftDrive.setPower(0);
        control.rightDrive.setPower(0);

    }

    public void initialise(){
        telemetry.addData("Status", "Initialized");
        control.gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        control.ball_color = hardwareMap.get(Servo.class,"ball_servo");
        control.ball_arm = hardwareMap.get(Servo.class,"ball_arm");

        control.leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        control.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        control.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        control.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("set up drive engines","");

        control.upDrive = hardwareMap.get(DcMotor.class, "up_drive");
        telemetry.addData("set up lifter and extender engines ","");

        control.grab_cube_left=hardwareMap.get(Servo.class,"grab_cube_left");
        control.grab_cube_right=hardwareMap.get(Servo.class,"grab_cube_right");
        telemetry.addData("set up grab servos","");

        control.leftDrive.setDirection(DcMotor.Direction.FORWARD);
        control.rightDrive.setDirection(DcMotor.Direction.REVERSE);
        control.upDrive.setDirection(DcMotor.Direction.FORWARD);

        sleep(2000);
    }

    public void autonomousmove(double TURN_SPEED,double seconds){
        TURN_SPEED_1=0.2;
        control.leftDrive.setPower(+TURN_SPEED_1);
        control.rightDrive.setPower(+TURN_SPEED_1);
        runtime.reset();
        runtime2.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            if(runtime2.seconds()>=0.1 && TURN_SPEED_1<=TURN_SPEED)
            {
                TURN_SPEED_1=TURN_SPEED_1+0.1;
                control.leftDrive.setPower(+TURN_SPEED_1);
                control.rightDrive.setPower(+TURN_SPEED_1);
                runtime2.reset();
            }
        }

        control.leftDrive.setPower(0);
        control.rightDrive.setPower(0);
    }

    public void autonomousup(double upspeed,double seconds){
        control.upDrive.setPower(upspeed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        control.upDrive.setPower(0);
    }

    @Override
    public void runOpMode() {

        initialise();
/*
        control.gyro.calibrate(
        while (control.gyro.isCalibrating())
            sleep(5);
        telemetry.addData("Gyro", "Calibrating");
        sleep(10);
*/
        waitForStart();

        while (opModeIsActive()) {

            turnAbsolute(90);
            sleep(2000);
            control.turnLeftByGyro(0.3,90);

            break;
        }
    }

}
