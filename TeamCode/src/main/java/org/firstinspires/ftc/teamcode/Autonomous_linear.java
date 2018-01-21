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
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;


@Autonomous(name="a_linear", group ="Autonomous")
public class Autonomous_linear extends LinearOpMode {

    VuforiaLocalizer vuforia;
    private ColorSensor colorSensor;
    private ColorSensor under_colorSensor;
    private controls control = new controls();
    private ModernRoboticsI2cGyro   gyro    = null;
    private String team_color="blue";
    private PushbotAutoDriveByGyro_Linear driveByGyro = new PushbotAutoDriveByGyro_Linear();

    @Override public void runOpMode() {

        //setting up motors
        telemetry.addData("Status", "Initialized");

        control.leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        control.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        control.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        control.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        control.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        control.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("set up drive engines","");

        control.upDrive = hardwareMap.get(DcMotor.class, "up_drive");
        control.extendDrive= hardwareMap.get(DcMotor.class, "extend_drive");
        telemetry.addData("set up lifter and extender engines ","");

        control.grab_cube_left=hardwareMap.get(Servo.class,"grab_cube_left");
        control.grab_cube_right=hardwareMap.get(Servo.class,"grab_cube_right");
        telemetry.addData("set up grab servos","");

        control.leftDrive.setDirection(DcMotor.Direction.FORWARD);
        control.rightDrive.setDirection(DcMotor.Direction.REVERSE);
        control.upDrive.setDirection(DcMotor.Direction.FORWARD);
        control.extendDrive.setDirection(DcMotor.Direction.FORWARD);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");


        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        under_colorSensor = hardwareMap.get(ColorSensor.class, "under_colorsensor");

        // vumark configuration
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AW9KAvX/////AAAAGSoAGMf4Dkz0hJ7OIMefI9w9qAkRHuDZBtDVnai4mtg/RUSwT94QTlOFFGJoaF55C1C+aponf8pYfTkVDKBGsGosyfQp1JQZvagKfsyLIYgs8pmZ7GYk7zCjZ1AN3mnmg8558Z/G7SwsaEgCJD2TLmsWYxaKe8PmDLPvRB57dJSJ30lhP9mhPoBmJo0futlynTkzNIn18MR0+DnCCbSIY3UPiwePzC3/AOZyEMV2mVfC/poxmEN+r1cbTCQ4fbjG6OgD0yS7yK9U3VhI97jJJ673neGOyBRJNQqvgdVT/SkjjnlCGVyYrk9nDmiqxQQq8Zju4/CkjodjuRnIBxhc2cWfNbVIQLOBl6LlL9c4Rnh9";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        //

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        control.grab();



        while (opModeIsActive()) {
            control.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            control.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            control.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            control.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("color values:", String.format("red: {0} green: {1} blue: {2}", colorSensor.red()),colorSensor.green(),colorSensor.blue());
            if(under_colorSensor.red()>under_colorSensor.blue()) {
                telemetry.addData("ball color: ", "red");
                team_color = "red";
            }



            else{team_color="blue";
                telemetry.addData("ball color: ","blue");
            }

            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            telemetry.addData(">", "Calibrating Gyro");    //

            telemetry.update();
            gyro.calibrate();
            while (!isStopRequested() && gyro.isCalibrating())  {
                sleep(50);
                idle();
            }



           /** if((colorSensor.red()>colorSensor.blue()))
            if(team_color=="red")
            {
                driveByGyro.gyroDrive(0.3,0.5,0);
                driveByGyro.gyroTurn(0.4,90);
            }
            else
            **/

            if(vuMark==RelicRecoveryVuMark.RIGHT){
                driveByGyro.gyroDrive(0.5,0.2,0);
                gyro.calibrate();
                driveByGyro.gyroTurn(0.2,90);
                driveByGyro.gyroDrive(0.5,0.2,0);
                control.grab();
                driveByGyro.gyroTurn(0.2,-90);
                driveByGyro.gyroDrive(-1,-0.3,0);
            }

            if(vuMark==RelicRecoveryVuMark.CENTER){
                driveByGyro.gyroDrive(0.5,0.5,0);
                gyro.calibrate();
                driveByGyro.gyroTurn(0.2,90);
                driveByGyro.gyroDrive(0.5,0.2,0);
                control.grab();
                driveByGyro.gyroTurn(0.2,-90);
                driveByGyro.gyroDrive(-1,-0.6,0);
            }
            if(vuMark==RelicRecoveryVuMark.LEFT){
                driveByGyro.gyroDrive(0.5,0.8,0);
                gyro.calibrate();
                driveByGyro.gyroTurn(0.2,90);
                driveByGyro.gyroDrive(0.5,0.2,0);
                control.grab();
                driveByGyro.gyroTurn(0.2,-90);
                driveByGyro.gyroDrive(-1,-0.9,0);
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
            telemetry.addData("Status", "X" + gyro.rawX());
            telemetry.addData("Status", "Y" + gyro.rawY());
            telemetry.addData("Status", "Z" + gyro.rawZ());
            telemetry.update();





            break;
        }
    }

}
