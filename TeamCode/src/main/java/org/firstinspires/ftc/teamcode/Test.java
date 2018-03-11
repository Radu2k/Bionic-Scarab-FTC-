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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Test", group ="Autonomous")
public class Test extends LinearOpMode {

    private ColorSensor color_sensor;
    private Controls control = new Controls();
    private double team_color ;

    VuforiaLocalizer vuforia;

    private ElapsedTime runtime = new ElapsedTime();

    public void initialise() {
        telemetry.addData("Status", "Initialized");
        color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        control.ball_servo = hardwareMap.get(Servo.class, "ball_servo");

        control.leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        control.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        control.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        control.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("set up drive engines", "");

        control.upDrive = hardwareMap.get(DcMotor.class, "up_drive");
        control.extendDrive = hardwareMap.get(DcMotor.class, "extend_drive");
        telemetry.addData("set up lifter and extender engines ", "");

        control.grab_cube_left = hardwareMap.get(Servo.class, "grab_cube_left");
        control.grab_cube_right = hardwareMap.get(Servo.class, "grab_cube_right");
        telemetry.addData("set up grab servos", "");

        control.leftDrive.setDirection(DcMotor.Direction.FORWARD);
        control.rightDrive.setDirection(DcMotor.Direction.REVERSE);
        control.upDrive.setDirection(DcMotor.Direction.FORWARD);
        control.extendDrive.setDirection(DcMotor.Direction.FORWARD);

        team_color = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();


        control.gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
    }

    public void autonomousmove(double TURN_SPEED,double seconds){
        control.leftDrive.setPower(+TURN_SPEED);
        control.rightDrive.setPower(+TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("voltage",team_color);
            telemetry.update();
        }
        control.leftDrive.setPower(0);
        control.rightDrive.setPower(0);

    }


    @Override
    public void runOpMode() {
        initialise();
        waitForStart();


        control.gyro.calibrate();
        while (control.gyro.isCalibrating())
            sleep(5);

        while (opModeIsActive()) {

            //autonomousmove(1,0.53);
            autonomousmove(0,control.AdjustForBattery(1,hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage()));
            //control.turnLeftByGyro(0.6,90);

            //autonomousmove(1,0.62);
            //autonomousmove(0,10);
            //autonomousmove(1,0.78);

            break;

        }


    }
}