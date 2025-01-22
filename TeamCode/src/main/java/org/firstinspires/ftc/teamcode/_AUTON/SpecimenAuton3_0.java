package org.firstinspires.ftc.teamcode._AUTON;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode._CONFIG.Hware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SpecimenAuton3_0 extends LinearOpMode {

    Boolean RUNNOTON = true;
    Boolean RUNNOTONV = true;

    double SPEEDCONTROL = 1;
    double TURNCONTROL = 1;
    String vMSG = "PICK";
    String hMSG = "PICK";
    Hware robot;
    int intakeWait = 0;
    double wristPos = 0;
    public void changeHorizontalPosition(int pos1, int pos2, double pow) {
        for(int i = 0; i < intakeWait; i++){

        }
        intakeWait = 0;
        RUNNOTON = false;
        robot.horizontal1.setTargetPosition(pos1);
        robot.horizontal2.setTargetPosition(pos2);
        robot.horizontal1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horizontal2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horizontal1.setPower(pow);
        robot.horizontal2.setPower(pow);
    }

    public void changeVerticalPosition(int pos1, int pos2, double pow) {
        robot.vertical1.setTargetPosition(pos1);
        robot.vertical2.setTargetPosition(pos2);
        robot.vertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertical2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertical1.setPower(pow);
        robot.vertical2.setPower(pow);
    }

    public void openIntakeClaw(){
        robot.intakeClaw.setPosition(0.2);
    }

    public void closeIntakeClaw(){
        robot.intakeClaw.setPosition(0.5);
    }

    public void openOuttakeClaw(){
        robot.outtakeClaw.setPosition(0.08);
    }

    public void closeOuttakeClaw(){
        robot.outtakeClaw.setPosition(0.25);
    }

    public void changeVerticalPosition(int Position1, int Position2) {
        RUNNOTONV = false;
        robot.vertical1.setTargetPosition(Position1);
        robot.vertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertical2.setTargetPosition(Position2);
        robot.vertical2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // INTAKE PORTION
    public void rotateIntakeToGround(boolean adjusting) {
        if(adjusting) {
            robot.rightElbow.setPosition(0.159);
            robot.leftElbow.setPosition(0.841);
        }else{
            robot.rightElbow.setPosition(0);
            robot.leftElbow.setPosition(1);
        }
    }

    public void rotateIntakeToTransfer() {
        robot.rightElbow.setPosition(0.55);
        robot.leftElbow.setPosition(0.45);

    }

    public void rotateIntakeToMove() {
        robot.rightElbow.setPosition(0.573);
        robot.leftElbow.setPosition(0.426);
        intakeWait = 0;
    }

    //Pick from wall: r: 0.873, l:0.127
    //Transfer: r: 0, l: 1


    public void rotateOuttakeToPickWall() {
        robot.rightNeck.setPosition(0.75);
        robot.leftNeck.setPosition(0.25);
        robot.jaw.setPosition(0.1);

    }

    public void rotateOuttakeToScoreBucket() {

        robot.rightNeck.setPosition(0.42);
        robot.leftNeck.setPosition(0.58);
        robot.jaw.setPosition(0.9);

    }

    public void rotateOuttakeToScoreBack() {
        robot.rightNeck.setPosition(0.67);
        robot.leftNeck.setPosition(0.33);
        robot.jaw.setPosition(0.1);

    }

    public void rotateOuttakeToScoreFront() {
        robot.rightNeck.setPosition(0);
        robot.leftNeck.setPosition(1);
        robot.jaw.setPosition(0.5);

    }

    public void rotateOuttakeToTransfer() {
        robot.rightNeck.setPosition(0);
        robot.leftNeck.setPosition(1);
        robot.jaw.setPosition(0.708);
    }

    public void rotateOuttakeToHangSpecimen() {
        robot.rightNeck.setPosition(0.82);
        robot.leftNeck.setPosition(0.18);
        robot.jaw.setPosition(0.555);
    }



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Hware();
        robot.initialize(hardwareMap);

        int pushSpeed = 80;
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence holdPreload = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(0, ()->{
                    closeOuttakeClaw();
                })
                .lineToConstantHeading(new Vector2d(0, 0.1),null, new ProfileAccelerationConstraint(30))

                .build();

        TrajectorySequence preloadMove = drive.trajectorySequenceBuilder(holdPreload.end())
                .addTemporalMarker(0, ()->{
                    rotateOuttakeToScoreBack();
                    changeVerticalPosition(-360, 360, 0.5);
                })
                .lineToConstantHeading(new Vector2d(-28, -6),null, new ProfileAccelerationConstraint(30))
                .build();

        TrajectorySequence preloadScore = drive.trajectorySequenceBuilder(preloadMove.end())
                .addTemporalMarker(0, ()->{
                    rotateIntakeToTransfer();
                    changeVerticalPosition(0,0,0.8);
                })
                .lineToConstantHeading(new Vector2d(-28, -6.1),null, new ProfileAccelerationConstraint(30))

                .build();

        TrajectorySequence preloadScoreRelease = drive.trajectorySequenceBuilder(preloadScore.end())
                .addTemporalMarker(0, ()->{
                    openOuttakeClaw();
                    changeHorizontalPosition(0,0,0.8);
                })
                .lineToConstantHeading(new Vector2d(-28, -6.2),null, new ProfileAccelerationConstraint(30))
                .waitSeconds(0.1)
                .build();

        TrajectorySequence pushAll = drive.trajectorySequenceBuilder(preloadScoreRelease.end())
                .splineToConstantHeading(new Vector2d(-36,24), Math.toRadians(180),null, new ProfileAccelerationConstraint(pushSpeed))
                .addDisplacementMarker(()->{
                    changeVerticalPosition(0,0,0.5);
                    rotateOuttakeToPickWall();
                    openOuttakeClaw();
                })
                .lineToConstantHeading(new Vector2d(-53, 23),null, new ProfileAccelerationConstraint(pushSpeed))
                .lineToConstantHeading(new Vector2d(-53, 32),null, new ProfileAccelerationConstraint(pushSpeed))
                .lineToConstantHeading(new Vector2d(-10, 32),null, new ProfileAccelerationConstraint(pushSpeed))
                .lineToConstantHeading(new Vector2d(-53, 30),null, new ProfileAccelerationConstraint(pushSpeed))
                .lineToConstantHeading(new Vector2d(-53, 40),null, new ProfileAccelerationConstraint(pushSpeed))
                .lineToConstantHeading(new Vector2d(-10, 42),null, new ProfileAccelerationConstraint(pushSpeed))
                .build();

        TrajectorySequence alignToPick1 = drive.trajectorySequenceBuilder(pushAll.end())
                .lineToLinearHeading(new Pose2d(-10, 35, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-4.4,35),null, new ProfileAccelerationConstraint(15))
                .waitSeconds(0.05)
                .lineToConstantHeading(new Vector2d(-4.4001,35),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    closeOuttakeClaw();
                }).waitSeconds(0.05)
                .lineToConstantHeading(new Vector2d(-4.4002,35),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    changeVerticalPosition(-360, 360, 0.5);
                }).waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-4.4003,35),null, new ProfileAccelerationConstraint(30))
                .build();

        TrajectorySequence alignToScore1 = drive.trajectorySequenceBuilder(alignToPick1.end())
                .addTemporalMarker(0, ()->{
                    rotateOuttakeToScoreBack();
                })
                //.waitSeconds(0)
                .lineToLinearHeading(new Pose2d(-29, -8, -Math.toRadians(359)),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    changeVerticalPosition(0,0,0.8);
                })
                .build();

        TrajectorySequence score1 = drive.trajectorySequenceBuilder(alignToScore1.end())
                .lineToConstantHeading(new Vector2d(-29.001, -8),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    openOuttakeClaw();
                }).lineToConstantHeading(new Vector2d(-29.002, -8),null, new ProfileAccelerationConstraint(30))
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-29.003, -8),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    rotateOuttakeToPickWall();
                })
                .build();

        TrajectorySequence alignToPick2 = drive.trajectorySequenceBuilder(score1.end())
                .addTemporalMarker(0, ()->{
                    changeVerticalPosition(0,0,0.2);
                })
                .lineToLinearHeading(new Pose2d(-15, 35, Math.toRadians(180)))
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(-4.4,35),null, new ProfileAccelerationConstraint(15))
                .addDisplacementMarker(()->{
                    closeOuttakeClaw();
                }).waitSeconds(0.05)
                .lineToConstantHeading(new Vector2d(-4.4002,35),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    changeVerticalPosition(-360, 360, 0.5);
                }).waitSeconds(0.05)
                .lineToConstantHeading(new Vector2d(-4.4003,35),null, new ProfileAccelerationConstraint(30))
                .build();

        TrajectorySequence alignToScore2 = drive.trajectorySequenceBuilder(alignToPick2.end())
                .addTemporalMarker(0, ()->{
                    rotateOuttakeToScoreBack();
                })
                //.waitSeconds(0)
                .lineToLinearHeading(new Pose2d(-30, -12, -Math.toRadians(359)),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    changeVerticalPosition(0,0,0.8);
                })
                .build();

        TrajectorySequence score2 = drive.trajectorySequenceBuilder(alignToScore2.end())
                .lineToConstantHeading(new Vector2d(-30.001, -12),null, new ProfileAccelerationConstraint(30))
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-30.002, -12),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    openOuttakeClaw();
                }).lineToConstantHeading(new Vector2d(-30.001, -12),null, new ProfileAccelerationConstraint(30))
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-30.002, -12),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    //rotateOuttakeToPickWall();
                })
                .build();

        TrajectorySequence alignToPick3 = drive.trajectorySequenceBuilder(score2.end())
                .addTemporalMarker(0, ()->{
                    changeVerticalPosition(0,0,0.4);
                })
                .lineToConstantHeading(new Vector2d(-30.001, -12),null, new ProfileAccelerationConstraint(30))
                .waitSeconds(20)
                .lineToLinearHeading(new Pose2d(-10, 35, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-4,35),null, new ProfileAccelerationConstraint(15))
                .waitSeconds(0.05)
                .lineToConstantHeading(new Vector2d(-4.0001,35),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    closeOuttakeClaw();
                }).waitSeconds(0.05)
                .lineToConstantHeading(new Vector2d(-4.0002,35),null, new ProfileAccelerationConstraint(30))
                .addDisplacementMarker(()->{
                    changeVerticalPosition(-170,164,0.7);
                }).waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-4.0003,35),null, new ProfileAccelerationConstraint(30))
                .build();

        TrajectorySequence alignToScore3 = drive.trajectorySequenceBuilder(alignToPick3.end())
                .lineToConstantHeading(new Vector2d(-26, -6)).waitSeconds(1)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(alignToScore3.end())
                .lineToConstantHeading(new Vector2d(-5, 35))
                .build();

//        TrajectorySequence alignToPush3 = drive.trajectorySequenceBuilder(push2.end())
//                .lineToConstantHeading(new Vector2d(-64, 48))
//                .build();
//
//        TrajectorySequence push3 = drive.trajectorySequenceBuilder(alignToPush3.end())
//                .lineToConstantHeading(new Vector2d(-10, 50))
//                .build();

        closeOuttakeClaw();
        rotateOuttakeToTransfer();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(holdPreload);
        drive.followTrajectorySequence(preloadMove);
        drive.followTrajectorySequence(preloadScore);
        drive.followTrajectorySequence(preloadScoreRelease);
        drive.followTrajectorySequence(pushAll);
        drive.followTrajectorySequence(alignToPick1);
        drive.followTrajectorySequence(alignToScore1);
        drive.followTrajectorySequence(score1);
        drive.followTrajectorySequence(alignToPick2);
        drive.followTrajectorySequence(alignToScore2);
        drive.followTrajectorySequence(score2);
        drive.followTrajectorySequence(alignToPick3);
        drive.followTrajectorySequence(alignToScore3);
        drive.followTrajectorySequence(park);


//        drive.followTrajectorySequence(alignToPush3);
//        drive.followTrajectorySequence(push3);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
