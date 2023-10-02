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

package Tessts;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import CompCode.TELEmap;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//start position thingy Ltarget = 440 , Rtarget = 439

@Disabled
@TeleOp(name="COMP TELE???", group="COMP")

public class COM_TELE extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    TELEmap robot   = new TELEmap();

    double SpeedAdjust = 1;
    boolean isPressed = false;

    double SSVar = 5;

    double dropVar = 50;
    double liftVar = 350;


    public static int CONESTART = 655;
    public static int CONE5 = 570;
    public static int CONE4 = 515;



    double toggleTime = .25;
    ElapsedTime toggle = new ElapsedTime(); //STOP


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.leftClaw.setPosition(L_OPEN);




        telemetry.addData("Status", "Initialized");
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        START_POS();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive equation
            drive();



            if (gamepad1.right_trigger == 1){
                grab();
            } else if (gamepad1.left_trigger==1){
                robot.lin.setPower(-1);
                robot.rin.setPower(-1);
            }else  {
                robot.lin.setPower(0);
                robot.rin.setPower(0);
            }
            if (gamepad1.square) {
                lift(0);
            }





            if (gamepad2.dpad_up) {
                HIGH();
            } else if (gamepad2.dpad_left) {
                MID();
            } else if (gamepad2.dpad_down) {
                LOW();
            } else if (gamepad2.dpad_right) {
                MID();
            } else if (gamepad2.cross) {
                START_POS();
            } else if (gamepad2.square) {
                FLIP();
            }

            else if (gamepad2.triangle) {
                //start cone
                if (SSVar == 5){
                    //intake 5
                    lift(250);
                }
                else if (SSVar == 4) {
                    //intake 4
                    lift(210);
                }
                else if (SSVar == 3){
                    //intake 3
                    lift(170);

                }
                else if (SSVar==2){
                    //intake 2
                    lift(130);

                }
                else if (SSVar==1){
                    //intake 2
                    lift(90);

                }

            }

//            else if (gamepad2.right_trigger==1) {
//                grab();
//            }


            if (gamepad2.right_bumper) {
                toggleR();
            }
            else if( gamepad2.left_bumper) {
                toggleL();
            }

            telemetry.addData("SS", SSVar);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Endgame:", endgame);
            telemetry.update();
        }
    }


    //TODO: put in an auto
    public void lift(double counts) {
        int newTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newTarget = (int) counts;
            //inches *
            robot.larm.setTargetPosition(newTarget);
            robot.rarm.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.larm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.larm.setPower(Math.abs(-1)); //left arm positive
            robot.rarm.setPower(Math.abs(1)); //right arm negative

            while (opModeIsActive()
                    //&& (runtime.seconds() < timeoutS)
                    &&
                    (robot.larm.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d");
                telemetry.addData("Path2", "Running at %7d :%7d");
                telemetry.update();
                drive();
                grab();
            }

        }
    }

    public void toggleR (){
        runtime.reset();
        if (SSVar <5) {
            SSVar++;
        }
        while (opModeIsActive()
                && (runtime.seconds() < toggleTime)
        ) {
            // Display it for the driver.
            telemetry.addData("SS", SSVar);
            telemetry.update();
            drive();
        }
    }
    public void toggleL (){
        runtime.reset();
        if (SSVar >1) {
            SSVar--;
        }
        while (opModeIsActive()
                && (runtime.seconds() < toggleTime)
        ) {
            // Display it for the driver.
            telemetry.addData("SS", SSVar);
            telemetry.update();
            drive();
        }
    }

    public void grab (){
        robot.rin.setPower(1);
        robot.lin.setPower(1);
        //drop intake 180
        dropVar = robot.larm.getCurrentPosition() - 30;
        lift(dropVar);
        robot.rin.setPower(0);
        robot.lin.setPower(0);

        liftVar = robot.larm.getCurrentPosition() + 100;
        lift(liftVar);
        if (gamepad1.left_trigger==1){
            robot.lin.setPower(-1);
            robot.rin.setPower(-1); }
    }

    //655 = cone start intake height
    //cone 1 = 455
    //start 4 = 600

    public void START_POS(){
        lift(90);
    }

    public void LOW(){
        lift(460);
    }

    public void MID(){
        lift(780);
    }

    public void HIGH(){
        lift(1100);
    }



    public void FLIP(){
        lift(10);
    }

    private void drive() {
        robot.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
        robot.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
        robot.leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
        robot.rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

        if (gamepad1.left_bumper) {
            SpeedAdjust = 4;
        } else if (gamepad1.right_bumper) {
            SpeedAdjust = 1;
        }
    }


}