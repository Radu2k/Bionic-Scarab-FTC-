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


@Autonomous(name="Autonomous_linear_left_side_blue_team", group ="Autonomous")
public class Autonomous_linear_left_side_blue_team extends LinearOpMode {

    private ColorSensor color_sensor;
    private Controls control = new Controls();

    public DcMotor extendDrive = null;

    VuforiaLocalizer vuforia;

    static  double     TURN_SPEED_1  = 0.2;
    int zAccumulated;

    Servo ball_arm;
    Servo ball_color;
    Servo relicv_up;
    Servo relicv_grab;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();



    public void initialise(){
        telemetry.addData("Status", "Initialized");
        control.gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        ball_color = hardwareMap.get(Servo.class,"ball_servo");
        ball_arm = hardwareMap.get(Servo.class,"ball_arm");
        ball_arm.setPosition(1);
        ball_color.setPosition(0.5);

        control.leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        control.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        control.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        control.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("set up drive engines","");

        control.upDrive = hardwareMap.get(DcMotor.class, "up_drive");
        telemetry.addData("set up lifter and extender engines ","");
        extendDrive= hardwareMap.get(DcMotor.class, "extend_drive");

        control.grab_cube_left=hardwareMap.get(Servo.class,"grab_cube_left");
        control.grab_cube_left.setPosition(1);
        control.grab_cube_right=hardwareMap.get(Servo.class,"grab_cube_right");
        telemetry.addData("set up grab servos","");

        control.leftDrive.setDirection(DcMotor.Direction.FORWARD);
        control.rightDrive.setDirection(DcMotor.Direction.REVERSE);
        control.upDrive.setDirection(DcMotor.Direction.FORWARD);

        relicv_up=hardwareMap.get(Servo.class,"relicv_up");
        relicv_grab=hardwareMap.get(Servo.class,"relicv_grab");
        relicv_grab.setPosition(0.0);
        relicv_up.setPosition(0.0);
    }

    public void autonomousmove(double TURN_SPEED,double seconds){
        control.leftDrive.setPower(+TURN_SPEED);
        control.rightDrive.setPower(+TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        control.leftDrive.setPower(0);
        control.rightDrive.setPower(0);
    }

    public void turnAbsolute(int target,double turnSpeed,int i) {
        control.gyro.resetZAxisIntegrator();
        zAccumulated = control.gyro.getIntegratedZValue();  //Set variables to gyro readings


        while (Math.abs(zAccumulated - target) > i && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                control.leftDrive.setPower(turnSpeed);
                control.rightDrive.setPower(-(turnSpeed+0.1));
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                control.leftDrive.setPower(-turnSpeed);
                control.rightDrive.setPower(turnSpeed+0.1);
            }

            zAccumulated = control.gyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
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

    public void ball_control(char part) {
        if (part=='l')
        {ball_color.setPosition(0);
            sleep(500);
        }

        else
        {ball_color.setPosition(1);
            sleep(500);
        }

    }//bile

    @Override
    public void runOpMode() {


        RelicRecoveryVuMark vuMark = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AW9KAvX/////AAAAGSoAGMf4Dkz0hJ7OIMefI9w9qAkRHuDZBtDVnai4mtg/RUSwT94QTlOFFGJoaF55C1C+aponf8pYfTkVDKBGsGosyfQp1JQZvagKfsyLIYgs8pmZ7GYk7zCjZ1AN3mnmg8558Z/G7SwsaEgCJD2TLmsWYxaKe8PmDLPvRB57dJSJ30lhP9mhPoBmJo0futlynTkzNIn18MR0+DnCCbSIY3UPiwePzC3/AOZyEMV2mVfC/poxmEN+r1cbTCQ4fbjG6OgD0yS7yK9U3VhI97jJJ673neGOyBRJNQqvgdVT/SkjjnlCGVyYrk9nDmiqxQQq8Zju4/CkjodjuRnIBxhc2cWfNbVIQLOBl6LlL9c4Rnh9";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {
            initialise();

            control.gyro.calibrate();
            while (control.gyro.isCalibrating())
                sleep(5);
            telemetry.addData("Gyro", "Calibrating");
            sleep(10);

            relicTrackables.activate();
            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            extendDrive.setPower(0);

            control.grab();
            autonomousmove(0,1);//stai
            autonomousup(0.5,0.5);//ridica cubu'


            ball_arm.setPosition(0.5);
            sleep(500);
            ball_color.setPosition(1);
            sleep(500);
            ball_arm.setPosition(0.15);
            sleep(500);
            ball_color.setPosition(0.43);
            sleep(500);
            ball_arm.setPosition(0.2);
            sleep(500);
            ball_arm.setPosition(0);
            sleep(500);

            while (color_sensor.blue() == color_sensor.red())
            {       ball_color.setPosition(ball_color.getPosition() - 0.02);
                sleep(700);}

            if((color_sensor.red()>color_sensor.blue()))
                ball_control('l');//left
            else
                ball_control('r');//right

            ball_arm.setPosition(0.5);
            sleep(100);
            ball_color.setPosition(0.48);
            sleep(100);
            ball_arm.setPosition(1);


            while (vuMark == RelicRecoveryVuMark.UNKNOWN) {

                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }


            if(vuMark==RelicRecoveryVuMark.LEFT){
                turnAbsolute(90,0.24,13);
                sleep(100);
                autonomousmove(0.3,0.7);
                sleep(100);
                turnAbsolute(-90,0.4,10);
                sleep(100);
                autonomousmove(0.3,0.62);
                sleep(100);
                turnAbsolute(90,0.4,9);
                sleep(100);
                autonomousmove(0.24,0.5);
                sleep(100);
                control.grab();
                autonomousmove(-0.24,0.5);
                autonomousmove(0.25,0.5);
                autonomousmove(-0.25,0.4);

            }

            if(vuMark==RelicRecoveryVuMark.CENTER){
                turnAbsolute(90,0.24,13);
                sleep(100);
                autonomousmove(0.3,0.7);
                sleep(100);
                turnAbsolute(-90,0.4,10);
                sleep(100);
                autonomousmove(0.3,0.7);
                sleep(100);
                turnAbsolute(90,0.4,9);
                sleep(100);
                autonomousmove(0.24,0.5);
                sleep(100);
                control.grab();
                autonomousmove(-0.24,0.5);
                autonomousmove(0.25,0.5);
                autonomousmove(-0.25,0.4);


            }


            if(vuMark==RelicRecoveryVuMark.RIGHT){
                turnAbsolute(90,0.24,13);
                sleep(100);
                autonomousmove(0.3,0.9);
                sleep(100);
                turnAbsolute(-90,0.4,10);
                sleep(100);
                autonomousmove(0.3,1.45);
                sleep(100);
                turnAbsolute(90,0.4,9);
                sleep(100);
                autonomousmove(0.24,0.5);
                sleep(100);
                control.grab();
                autonomousmove(-0.24,0.5);
                autonomousmove(0.25,0.5);
                autonomousmove(-0.25,0.4);
            }

            autonomousup(-0.5,0.5);
            autonomousup(0,0.1);

            break;
        }
    }

}
