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


@Autonomous(name="Autonomous_linear_left_side_red_team", group ="Autonomous")
public class Autonomous_linear_left_side_red_team extends LinearOpMode {

    private ColorSensor color_sensor;
    private Controls control = new Controls();



    VuforiaLocalizer vuforia;



    private ElapsedTime runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.3;
    static final double     TURN_SPEED    = 0.3;


    public void initialise(){
        telemetry.addData("Status", "Initialized");
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



        control.gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
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



        // vumark configuration
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


        control.gyro.calibrate();
        while (control.gyro.isCalibrating())
            sleep(5);
        telemetry.addData("Gyro", "Calibrating");
        sleep(10);



        while (opModeIsActive()) {
            relicTrackables.activate();
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            control.grab();
            control.grab();
            telemetry.addData("color values:", String.format("red: {0} green: {1} blue: {2}", color_sensor.red()),color_sensor.green(),color_sensor.blue());
            telemetry.update();

            autonomousmove(0,1);//stai
            autonomousup(0.5,1);//ridica cubu'

/*
            control.ball_arm.setPosition(-1);
            sleep(500);
            control.ball_arm.setPosition(0.5);
            sleep(500);
/*
            if(color_sensor.red()>color_sensor.blue()) {
                telemetry.addData("ball color: ", "red");
            }
            else{
                telemetry.addData("ball color: ","blue");
            }
            telemetry.update();


            if((color_sensor.red()>color_sensor.blue()))
                control.ball_control('l');//left
            else
                control.ball_control('r');//right

          //  control.ball_arm.setPosition(1);
           // sleep(500);
           // control.ball_arm.setPosition(0.5);
          //  sleep(500);
*/
            while (vuMark == RelicRecoveryVuMark.UNKNOWN) {

                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }


            if(vuMark==RelicRecoveryVuMark.RIGHT){
                telemetry.addData("DETECTED:","right");
            }
            if(vuMark==RelicRecoveryVuMark.CENTER){
                telemetry.addData("DETECTED:","center");
            }
            if(vuMark==RelicRecoveryVuMark.LEFT){
                telemetry.addData("DETECTED:","left");
            }
            telemetry.update();


            if(vuMark==RelicRecoveryVuMark.LEFT){
                autonomousmove(0,1);
                control.turnRightByGyro(0.3,90);
                autonomousmove(0,1);
                autonomousmove(0.3,1);
                autonomousmove(0,1);
                control.turnRightByGyro(0.38,92);
                autonomousmove(0,1);
                autonomousmove(0.3,0.5);
                autonomousmove(0,1);
                control.grab();

                autonomousmove(0,1);
                autonomousup(1,0.40);
                control.upDrive.setPower(0);
                autonomousmove(0,1);
                autonomousmove(-0.3,0.8);
                autonomousmove(+0.3,0.9);
                autonomousmove(-0.3,0.9);
                autonomousmove(0,1);
                control.turnLeftByGyro(0.38,180);
                autonomousmove(0,1);
                autonomousup(-1,1);


            }

            if(vuMark==RelicRecoveryVuMark.CENTER){
                autonomousmove(0,1);
                control.turnRightByGyro(0.3,90);
                autonomousmove(0,1);
                autonomousmove(0.3,1.14);
                autonomousmove(0,1);
                control.turnRightByGyro(0.38,92);
                autonomousmove(0,1);
                autonomousmove(0.3,0.5);
                autonomousmove(0,1);
                control.grab();

                autonomousmove(0,1);
                autonomousup(1,0.40);
                control.upDrive.setPower(0);
                autonomousmove(0,1);
                autonomousmove(-0.3,0.8);
                autonomousmove(+0.3,0.9);
                autonomousmove(-0.3,0.9);
                autonomousmove(0,1);
                control.turnLeftByGyro(0.38,180);
                autonomousmove(0,1);
                autonomousup(-1,1);


            }


            if(vuMark==RelicRecoveryVuMark.RIGHT){
                autonomousmove(0,1);
                control.turnRightByGyro(0.3,90);
                autonomousmove(0,1);
                autonomousmove(0.3,1.32);
                autonomousmove(0,1);
                control.turnRightByGyro(0.38,92);
                autonomousmove(0,1);
                autonomousmove(0.3,0.5);
                autonomousmove(0,1);
                control.grab();

                autonomousmove(0,1);
                autonomousup(1,0.40);
                control.upDrive.setPower(0);
                autonomousmove(0,1);
                autonomousmove(-0.3,0.8);
                autonomousmove(+0.3,0.9);
                autonomousmove(-0.3,0.9);
                autonomousmove(0,1);
                control.turnLeftByGyro(0.38,180);
                autonomousmove(0,1);
                autonomousup(-1,1);
                sleep(50000);
            }


            telemetry.addData("Status", "X" + control.gyro.rawX());
            telemetry.addData("Status", "Y" + control.gyro.rawY());
            telemetry.addData("Status", "Z" + control.gyro.rawZ());
            telemetry.update();

            break;
        }
    }

}
