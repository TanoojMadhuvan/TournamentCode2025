package org.firstinspires.ftc.teamcode._TELEOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._CONFIG.Hware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.opencv.core.Mat;

@TeleOp
public class DriverControlsV2 extends LinearOpMode {
    Boolean RUNNOTON = true;
    Boolean RUNNOTONV = true;

    boolean strafeonly = false;
    double SPEEDCONTROL = 1;
    double TURNCONTROL = 1;
    String vMSG = "PICK";
    String hMSG = "PICK";
    Hware robot = new Hware();

    public void changeHorizontalPosition(int Position) {
        RUNNOTON = false;
        robot.horizontal1.setTargetPosition(Position);
        robot.horizontal2.setTargetPosition(Position);
        robot.horizontal1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horizontal2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void changeVerticalPosition(int Position1, int Position2) {
        RUNNOTONV = false;
        robot.vertical1.setTargetPosition(Position1);
        robot.vertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertical2.setTargetPosition(Position2);
        robot.vertical2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // INTAKE PORTION
    public void rotateToDrop() {
        robot.rightElbow.setPosition(0.1);
        robot.leftElbow.setPosition(.9);
    }

    public void rotateToPick() {
        robot.rightElbow.setPosition(1);
        robot.leftElbow.setPosition(0);
    }

    public void openPickUpClaw() {
        robot.intakeClaw.setPosition(.4);
    }

    public void closePickUpClaw() {

        robot.intakeClaw.setPosition(1);
    }

    // OUTAKE PORTION
    public void transferRotateToDrop() {
//        robot.vRot1.setPosition(0.3);
//        robot.vRot2.setPosition(0.7);
//        robot.clawRot.setPosition(0);
    }

    public void transferRotateToPick() {
//        robot.vRot1.setPosition(.9);
//        robot.vRot2.setPosition(.1);
//        robot.clawRot.setPosition(1);
    }

    public void transferOpenClaw() {

//        robot.vClaw.setPosition(0);
    }

    public void transferCloseClaw()
    {
//        robot.vClaw.setPosition(1);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);
        GamepadEx driverOp;
        GamepadEx armOp;
        SampleMecanumDrive driveR = new SampleMecanumDrive(hardwareMap);
        driveR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MecanumDrive drive = new MecanumDrive(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);
        armOp = new GamepadEx(gamepad2);
        driverOp = new GamepadEx(gamepad1);
        double x,y,xy;

        robot.backRight.setInverted(true);
        robot.frontRight.setInverted(true);


//        // AUTOMATION PROGRAMMING
//        TrajectorySequence pickUp = driveR.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
//                .waitSeconds(.01)
//                .addDisplacementMarker(() -> {
//                    closePickUpClaw();
//                })
//                .build();
//        TrajectorySequence transfer = driveR.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
//                .waitSeconds(.01)
//
//                .addDisplacementMarker(() -> {
//                    changeHorizontalPosition(3);
//                    robot.horizontal1.setPower(1);
//                    robot.horizontal2.setPower(1);
//
//                    transferCloseClaw();
//                    openPickUpClaw();
//                })
//                .build();


        // Servo Presets
        transferCloseClaw();
        transferRotateToDrop();
        closePickUpClaw();
        rotateToPick();

        waitForStart();


        while(opModeIsActive()) {

            // DRIVER

            x = driverOp.getLeftX() * SPEEDCONTROL;
            y = driverOp.getLeftY() * SPEEDCONTROL;
            xy = driverOp.getRightX() * TURNCONTROL;

            y = gamepad1.square ? 0 : y; //Perfect straight
            x = gamepad1.cross ? 0 : x; //Perfect straight

            if (gamepad1.left_trigger > 0.5) {SPEEDCONTROL = .4;TURNCONTROL=.4;} else if(gamepad1.right_trigger>0.5) {TURNCONTROL = 0.4; SPEEDCONTROL = 1;} else {SPEEDCONTROL=1; TURNCONTROL=1;}

            drive.driveRobotCentric(
                    x,
                    y,
                    xy
            );

            // ARM

            // HORIZONTAL
//            if (armOp.getLeftY() != 0) {
//                RUNNOTON = true;
//                robot.horizontal1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.horizontal2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                if(robot.horizontal1.getCurrentPosition() < 1675 && robot.horizontal1.getCurrentPosition() > -6) {
//                    robot.horizontal1.setPower(armOp.getLeftY()/1);
//                    robot.horizontal2.setPower(armOp.getLeftY()/1);
//                } else {
//                    robot.horizontal1.setPower(0);
//                    robot.horizontal2.setPower(0);
//                }
//            } else {
//                if (RUNNOTON) {
//                    robot.horizontal1.setPower(0);
//                    robot.horizontal2.setPower(0);
//                }
//            }
//            if (gamepad2.triangle) {
//                changeHorizontalPosition(3);
//                robot.horizontal1.setPower(1);
//                robot.horizontal2.setPower(1);
//                hMSG = "PICK";
//
//            } else if (gamepad2.circle) {
//                changeHorizontalPosition(290);
//                robot.horizontal1.setPower(1);
//                robot.horizontal2.setPower(1);
//                hMSG = "DROP";
//
//            } else if (gamepad2.cross) {
//                changeHorizontalPosition(1670);
//                robot.horizontal1.setPower(1);
//                robot.horizontal2.setPower(1);
//                hMSG = "DROP";
//            }
//
//            // VERTICAL
//
//            if (armOp.getRightY() != 0) {
//                RUNNOTONV = true;
//                robot.vertical1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.vertical2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                if(robot.vertical2.getCurrentPosition() < 4400 && robot.vertical2.getCurrentPosition() > -16) {
//                    robot.vertical1.setPower(armOp.getRightY());
//                    robot.vertical2.setPower(-armOp.getRightY());
//                } else {
//                    robot.vertical1.setPower(0);
//                    robot.vertical2.setPower(0);
//                }
//            } else {
//                if (RUNNOTONV) {
//                    robot.vertical1.setPower(0);
//                    robot.vertical2.setPower(0);
//                }
//            }
//            if (gamepad2.dpad_down) {
//                changeVerticalPosition(0,0);
//                robot.vertical1.setPower(0.8);
//                robot.vertical2.setPower(0.8);
//                vMSG = "PICK";
//            } else if (gamepad2.dpad_right) {
//                changeVerticalPosition(-2627,2625);
//                robot.vertical1.setPower(0.8);
//                robot.vertical2.setPower(0.8);
//                vMSG = "DROP";
//            } else if (gamepad2.dpad_up) {
//                changeVerticalPosition(-4093,4171);
//                robot.vertical1.setPower(0.8);
//                robot.vertical2.setPower(0.8);
//                vMSG = "DROP";
//            }
//
//
//

            // SERVOS

//            if (vMSG == "PICK") {
//                transferRotateToPick();
//            } else if (vMSG == "DROP") {
//                transferRotateToDrop();
//            }

            //0, 0
            //-2627,2625
            //-4093,4071
//            if(gamepad2.share){
//                changeVerticalPosition(-2627, 2625);
//                robot.vertical1.setPower(0.1);
//                robot.vertical2.setPower(0.1);
//            }

//            if (gamepad2.dpad_down) {
//               changeVerticalPosition(0,0);
//               robot.vertical1.setPower(-0.3);
//               robot.vertical2.setPower(0.3);
//               vMSG = "PICK";
//           } else if (gamepad2.dpad_right) {
//                changeVerticalPosition(-2627,2625);
//                robot.vertical1.setPower(-0.3);
//                robot.vertical2.setPower(0.3);
//               vMSG = "DROP";
//           } else if (gamepad2.dpad_up) {
//                changeVerticalPosition(-4093,4071);
//                robot.vertical1.setPower(-0.3);
//                robot.vertical2.setPower(0.3);
//               vMSG = "DROP";
//           }
//
//            if (armOp.getRightY() != 0) {
//                RUNNOTONV = true;
//                robot.vertical1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.vertical2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                if(robot.vertical1.getCurrentPosition() < 7700 && robot.vertical1.getCurrentPosition() > -16) {
//                    robot.vertical1.setPower(-armOp.getRightY());
//                    robot.vertical2.setPower(armOp.getRightY());
//                } else {
//                    robot.vertical1.setPower(0);
//                    robot.vertical2.setPower(0);
//                }
//            }


//            if(gamepad2.square){
//                transferRotateToDrop();
//            }else{
//                transferRotateToPick();
//            }
//
//            if (hMSG == "PICK") {
//                rotateToPick();
//            } else if (hMSG == "DROP") {
//                rotateToDrop();
//            }

//            if (vMSG == "PICK") {
//                transferRotateToPick();
//            } else if (vMSG == "DROP") {
//                transferRotateToDrop();
//            }
//
//            if (gamepad2.left_bumper) {
//                openPickUpClaw();
//            } else {
//                closePickUpClaw();
//
//            }



            if (gamepad2.right_bumper) { transferCloseClaw(); } else {  transferOpenClaw();}

            telemetry.addData("H1 Encoder: ", (Integer.toString(robot.horizontal1.getCurrentPosition())));
            telemetry.addData("H2 Encoder: ", (Integer.toString(robot.horizontal2.getCurrentPosition())));
            telemetry.addData("V1 Encoder: ", (Integer.toString(robot.vertical1.getCurrentPosition())));
            telemetry.addData("V2 Encoder: ", (Integer.toString(robot.vertical2.getCurrentPosition())));
            telemetry.update();

        }

        while (!isStopRequested() && opModeIsActive());
    }
}
