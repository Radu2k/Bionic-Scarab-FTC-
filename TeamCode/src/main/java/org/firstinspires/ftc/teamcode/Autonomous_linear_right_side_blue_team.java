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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Autonomous_linear_right_side_blue_team", group ="Autonomous")
public class Autonomous_linear_right_side_blue_team extends LinearOpMode {

    private ColorSensor color_sensor;
    private Controls control = new Controls();

    VuforiaLocalizer vuforia;



    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.3;
    static  double     TURN_SPEED_1  = 0.2;
    int zAccumulated;

    public void initialise(){

            control.gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
            color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
            control.ball_color = hardwareMap.get(Servo.class, "ball_servo");
            control.ball_arm = hardwareMap.get(Servo.class, "ball_arm");

            control.leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
            control.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            control.leftDrive.getMotorType();

            control.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
            control.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            control.rightDrive.getMotorType();

            control.upDrive = hardwareMap.get(DcMotor.class, "up_drive");
            control.upDrive.getMotorType();

            control.grab_cube_left = hardwareMap.get(Servo.class, "grab_cube_left");
            control.grab_cube_right = hardwareMap.get(Servo.class, "grab_cube_right");


            control.leftDrive.setDirection(DcMotor.Direction.FORWARD);
            control.rightDrive.setDirection(DcMotor.Direction.REVERSE);
            control.upDrive.setDirection(DcMotor.Direction.FORWARD);
            control.grab();
        }


    public void turnAbsolute(int target,double turnSpeed,int i) {
        zAccumulated = control.gyro.getIntegratedZValue();  //Set variables to gyro readings


        while (Math.abs(zAccumulated - target) > i && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
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


    public void autonomousmove(double TURN_SPEED,double seconds){
        if (TURN_SPEED<0)
            TURN_SPEED_1=-0.2;
        else
            TURN_SPEED_1=0.2;

        control.leftDrive.setPower(+TURN_SPEED_1);
        control.rightDrive.setPower(+TURN_SPEED_1);
        runtime.reset();
        runtime2.reset();

        while (opModeIsActive() && (runtime.seconds() < seconds)) {

            if(runtime2.seconds()>=0.1 && TURN_SPEED_1<=TURN_SPEED && TURN_SPEED_1>0)
            {
                TURN_SPEED_1=TURN_SPEED_1+0.1;
                control.leftDrive.setPower(+TURN_SPEED_1);
                control.rightDrive.setPower(+TURN_SPEED_1);
                runtime2.reset();
            }
            else
            {if(runtime2.seconds()>=0.1 && TURN_SPEED_1>=TURN_SPEED && TURN_SPEED_1<0)
                {
                    TURN_SPEED_1=TURN_SPEED_1-0.1;
                    control.leftDrive.setPower(+TURN_SPEED_1);
                    control.rightDrive.setPower(+TURN_SPEED_1);
                    runtime2.reset();
                }
            }

        }

        control.leftDrive.setPower(0);
        control.rightDrive.setPower(0);
    }


    public void autonomousup(double upspeed,double seconds){
        control.upDrive.setPower(upspeed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {

        }
        control.upDrive.setPower(0);
    }

    @Override
    public void runOpMode() {

        initialise();
        control.ball_arm.setPosition(1);
        control.ball_color.setPosition(0.5);

        RelicRecoveryVuMark vuMark = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AW9KAvX/////AAAAGSoAGMf4Dkz0hJ7OIMefI9w9qAkRHuDZBtDVnai4mtg/RUSwT94QTlOFFGJoaF55C1C+aponf8pYfTkVDKBGsGosyfQp1JQZvagKfsyLIYgs8pmZ7GYk7zCjZ1AN3mnmg8558Z/G7SwsaEgCJD2TLmsWYxaKe8PmDLPvRB57dJSJ30lhP9mhPoBmJo0futlynTkzNIn18MR0+DnCCbSIY3UPiwePzC3/AOZyEMV2mVfC/poxmEN+r1cbTCQ4fbjG6OgD0yS7yK9U3VhI97jJJ673neGOyBRJNQqvgdVT/SkjjnlCGVyYrk9nDmiqxQQq8Zju4/CkjodjuRnIBxhc2cWfNbVIQLOBl6LlL9c4Rnh9";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();
        waitForStart();


        control.gyro.calibrate();
        while (control.gyro.isCalibrating())
            sleep(5);

        sleep(10);



        while (opModeIsActive()) {

            vuMark = RelicRecoveryVuMark.from(relicTemplate);


            control.grab();

            sleep(1000);//stai
            autonomousup(0.5,0.5);//ridica cubu'

            control.ball_arm.setPosition(0.5);
            sleep(1000);
            control.ball_color.setPosition(1);
            sleep(1000);
            control.ball_arm.setPosition(0.15);
            sleep(1000);
            control.ball_color.setPosition(0.5);
            sleep(1000);
            control.ball_arm.setPosition(0);
            sleep(1000);

            if((color_sensor.red()<color_sensor.blue()))
                control.ball_control('l');//left
            else
                control.ball_control('r');//right

            control.ball_arm.setPosition(0.5);
            sleep(1000);
            control.ball_color.setPosition(0.5);
            sleep(1000);
            control.ball_arm.setPosition(1);

/*
            while (vuMark == RelicRecoveryVuMark.UNKNOWN) {

                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }
*/

            //if(vuMark==RelicRecoveryVuMark.LEFT){
                sleep(1000);
                turnAbsolute(90,0.3,5);
                sleep(1000);
                autonomousmove(0.3,1);
                sleep(1000);
                turnAbsolute(90,0.38,4);
                sleep(1000);
                autonomousmove(0.3,1);
                sleep(1000);
                control.grab();

                sleep(1000);
                autonomousmove(-0.3,0.8);
                autonomousmove(+0.3,0.9);
                autonomousmove(-0.3,0.9);
                sleep(1000);
                turnAbsolute(-90,0.38,4);
                sleep(1000);
                autonomousup(-1,0.3);




         /* if(vuMark==RelicRecoveryVuMark.CENTER){

                sleep(1000);
                turnAbsolute(90,0.3,5);
                sleep(1000);
                autonomousmove(0.3,1.4);
                sleep(1000);
                turnAbsolute(90,0.38,4);
                sleep(1000);
                autonomousmove(0.3,1);
                sleep(1000);
                control.grab();

                sleep(1000);
                autonomousmove(-0.3,0.8);
                autonomousmove(+0.3,0.9);
                autonomousmove(-0.3,0.9);
                sleep(1000);
                turnAbsolute(-90,0.38,4);
                sleep(1000);
                autonomousup(-1,0.3);


            }


            if(vuMark==RelicRecoveryVuMark.RIGHT){
                sleep(1000);
                turnAbsolute(90,0.3,5);
                sleep(1000);
                autonomousmove(0.3,1.7);
                sleep(1000);
                turnAbsolute(90,0.38,4);
                sleep(1000);
                autonomousmove(0.3,1);
                sleep(1000);
                control.grab();

                sleep(1000);
                autonomousmove(-0.3,0.8);
                autonomousmove(+0.3,0.9);
                autonomousmove(-0.3,0.9);
                sleep(1000);
                turnAbsolute(-90,0.38,4);
                sleep(1000);
                autonomousup(-1,0.3);

            }
*/
            sleep(1000);


            break;
        }
    }

}
