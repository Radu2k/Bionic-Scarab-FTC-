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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="a_linear", group ="Autonomous")
public class Autonomous_linear_right_side extends LinearOpMode {

    private ColorSensor under_colorSensor;
    private Controls control = new Controls();
    private String team_color="blue";

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
    VuforiaTrackable relicTemplate = relicTrackables.get(0);

    private ElapsedTime runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

   public void initialise(){
        telemetry.addData("Status", "Initialized");
        under_colorSensor = hardwareMap.get(ColorSensor.class, "under_colorsensor");

        control.leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        control.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        control.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        control.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        control.gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
    }

    public void autonomousturnright(double TURN_SPEED,double seconds){
        control.leftDrive.setPower(+TURN_SPEED);
        control.rightDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void autonomousturnleft(double TURN_SPEED,double seconds){
        control.leftDrive.setPower(-TURN_SPEED);
        control.rightDrive.setPower(+TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void autonomousmove(double TURN_SPEED,double seconds){
        control.leftDrive.setPower(+TURN_SPEED);
        control.rightDrive.setPower(+TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void vuforia(RelicRecoveryVuMark vuMark1)
    {
        // vumark configuration
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AW9KAvX/////AAAAGSoAGMf4Dkz0hJ7OIMefI9w9qAkRHuDZBtDVnai4mtg/RUSwT94QTlOFFGJoaF55C1C+aponf8pYfTkVDKBGsGosyfQp1JQZvagKfsyLIYgs8pmZ7GYk7zCjZ1AN3mnmg8558Z/G7SwsaEgCJD2TLmsWYxaKe8PmDLPvRB57dJSJ30lhP9mhPoBmJo0futlynTkzNIn18MR0+DnCCbSIY3UPiwePzC3/AOZyEMV2mVfC/poxmEN+r1cbTCQ4fbjG6OgD0yS7yK9U3VhI97jJJ673neGOyBRJNQqvgdVT/SkjjnlCGVyYrk9nDmiqxQQq8Zju4/CkjodjuRnIBxhc2cWfNbVIQLOBl6LlL9c4Rnh9";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        //
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        vuMark1=vuMark;

    }


    @Override public void runOpMode() {

        initialise();

        RelicRecoveryVuMark vuMark = null;

        vuforia(vuMark);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        control.grab();

        control.gyro.calibrate();
        telemetry.addData("Gyro", "Calibrating");
        sleep(10);




        while (opModeIsActive()) {

            telemetry.addData("color values:", String.format("red: {0} green: {1} blue: {2}", under_colorSensor.red()),under_colorSensor.green(),under_colorSensor.blue());
            if(under_colorSensor.red()>under_colorSensor.blue()) {
                telemetry.addData("ball color: ", "red");
                team_color = "red";
            }


            else{team_color="blue";
                telemetry.addData("ball color: ","blue");
            }

            vuMark = RelicRecoveryVuMark.from(relicTemplate);


            /** if((colorSensor.red()>colorSensor.blue()))
             if(team_color=="red")
             {
                 control.gyroDrive(0.3,0.5,0);
                 control.gyroTurn(0.4,90);
             }
             else
             **/

            if(vuMark==RelicRecoveryVuMark.RIGHT){
                telemetry.addData("DETECTED:","right");
            }
            if(vuMark==RelicRecoveryVuMark.CENTER){
                telemetry.addData("DETECTED:","center");
            }
            if(vuMark==RelicRecoveryVuMark.LEFT){
                telemetry.addData("DETECTED:","left");
            }

            if(vuMark==RelicRecoveryVuMark.RIGHT){
                autonomousmove(FORWARD_SPEED,2);
                autonomousturnright(TURN_SPEED,1.3);
                control.grab();
                autonomousturnleft(TURN_SPEED,1.3);
                autonomousmove(-1,2);


            }

            if(vuMark==RelicRecoveryVuMark.CENTER){
                autonomousmove(FORWARD_SPEED,3);
                autonomousturnright(TURN_SPEED,1.3);
                control.grab();
                autonomousturnleft(TURN_SPEED,1.3);
                autonomousmove(-1,3);
            }

            if(vuMark==RelicRecoveryVuMark.LEFT){
                autonomousmove(FORWARD_SPEED,4);
                autonomousturnright(TURN_SPEED,1.3);
                control.grab();
                autonomousturnleft(TURN_SPEED,1.3);
                autonomousmove(-1,4);
            }



            telemetry.addData("Status", "X" + control.gyro.rawX());
            telemetry.addData("Status", "Y" + control.gyro.rawY());
            telemetry.addData("Status", "Z" + control.gyro.rawZ());
            telemetry.update();

            break;
        }
    }

}