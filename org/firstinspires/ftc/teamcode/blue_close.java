package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

//@Disabled
@Autonomous(name = "blue_close", group = "Blue")
public class blue_close extends LinearOpMode
{
    MarkerDeterminationPipeline pipeline;
    OpenCvWebcam webcam;

    Hardware_robodux23 robot = new Hardware_robodux23();

    IMU imu=null;

    boolean done = false;
    int markerpos = 3;
    double rightservoin = .72;
    double leftservoin = .28;
    double rightservoout = .2;
    double leftservoout = .8;



    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new MarkerDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        robot.init(hardwareMap);

        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        double xRotation = 90;
        double yRotation = 90;
        double zRotation = 27;
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        /*imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = DEGREES;
        imu.initialize(parameters);*/

        robot.intake_fold.setPosition(.21);
        robot.release.setPosition(.49);

        /*telemetry.addLine("Waiting for start");
        telemetry.update();


        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }*/
        waitForStart();
        //Start of Code
        robot.intake_fold.setPosition(.5);
        robot.pivot_two.setPosition(leftservoin);
        robot.pivot_one.setPosition(rightservoin);
        sleep(800);
        if (pipeline.getAnalysis() == marker_detection.MarkerDeterminationPipeline.MarkerPosition.RIGHT) {
            markerpos = 3;
        }
        if (pipeline.getAnalysis() == marker_detection.MarkerDeterminationPipeline.MarkerPosition.CENTER) {
            markerpos = 2;
        }
        if (pipeline.getAnalysis() == marker_detection.MarkerDeterminationPipeline.MarkerPosition.LEFT) {
            markerpos = 1;
        }
        if (pipeline.getAnalysis() == marker_detection.MarkerDeterminationPipeline.MarkerPosition.RIGHT) {
            markerpos = 3;
        }
        if (pipeline.getAnalysis() == marker_detection.MarkerDeterminationPipeline.MarkerPosition.CENTER) {
            markerpos = 2;
        }
        if (pipeline.getAnalysis() == marker_detection.MarkerDeterminationPipeline.MarkerPosition.LEFT) {
            markerpos = 1;
        }

        robot.intake_fold.setPosition(.21);
        //left
        if (markerpos == 1) {
            Backward(340,.4,0,120,190);
            sleep(100);
            StrafeRight(480,.3,0,120,190);
            sleep(100);
            Backward(340,.4,0,120,190);
            robot.intake.setPower(-1);
            sleep(400);
            Forward(140,.4,0,25,65);
            robot.intake.setPower(0);
            sleep(100);
            TurnRight(90,.55,0,10,60);
            sleep(100);
            Forward(1175,.6,90,300,600);
            sleep(100);
            robot.pivot_two.setPosition(leftservoout);
            robot.pivot_one.setPosition(rightservoout);
            robot.lift.setTargetPosition(375);
            robot.lift.setPower(1);
            StrafeRight(190,.3,90,50,120);
            sleep(100);
            robot.lf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.lf_drive.setPower(.15);
            robot.rf_drive.setPower(.15);
            robot.lr_drive.setPower(.15);
            robot.rr_drive.setPower(.15);
            sleep(1200);

            robot.release.setPosition(.53);
            sleep(300);
            robot.lift.setTargetPosition(950);
            sleep(300);
            Backward(100,.4,90,25,60);
            robot.pivot_two.setPosition(leftservoin);
            robot.pivot_one.setPosition(rightservoin);
            robot.lift.setTargetPosition(0);
            StrafeRight(1425,.6,90,325,575);
            sleep(100);
            Backward(4475,.85,90,400,1050);
            robot.intake.setPower(1);
            robot.intake_fold.setPosition(.39);
            sleep(600);
            Backward(75,.13,90,20, 50);
            sleep(400);
            Forward(220,.2,90,25,50);
            robot.intake_fold.setPosition(.5);
            sleep(400);
            robot.intake_fold.setPosition(.21);
            sleep(600);
            robot.intake_fold.setPosition(.5);
            sleep(400);
            robot.release.setPosition(.49);
            sleep(200);
            robot.intake.setPower(-1);
            Forward(4230,.85,90,400,900);
            robot.pivot_two.setPosition(leftservoout);
            robot.pivot_one.setPosition(rightservoout);
            robot.lift.setTargetPosition(800);
            StrafeLeft(850,.6,90,300,500);
            robot.intake.setPower(0);
            sleep(100);
            robot.lf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.lf_drive.setPower(.15);
            robot.rf_drive.setPower(.15);
            robot.lr_drive.setPower(.15);
            robot.rr_drive.setPower(.15);
            sleep(1700);
            robot.release.setPosition(.53);
            sleep(200);
            robot.lift.setTargetPosition(1300);
            sleep(300);
            Backward(100,.3,90,25,30);
            robot.pivot_two.setPosition(leftservoin);
            robot.pivot_one.setPosition(rightservoin);
            robot.lift.setTargetPosition(0);
            sleep(1000);
        }

        //middle
        if (markerpos == 2) {
            Backward(1250, .57, 0, 300, 700);
            sleep(100);
            Forward(40,.3,0,15,25);
            robot.intake.setPower(-1);
            sleep(400);
            Forward(245,.4,0,60,150);
            robot.intake.setPower(0);
            sleep(100);
            TurnRight(90,.55,0,10,60);
            robot.pivot_two.setPosition(leftservoout);
            robot.pivot_one.setPosition(rightservoout);
            robot.lift.setTargetPosition(375);
            robot.lift.setPower(1);
            StrafeRight(80,.3,90,35,60);
            sleep(100);
            Forward(1540,.6,90,350,700);
            sleep(100);
            robot.lf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.lf_drive.setPower(.15);
            robot.rf_drive.setPower(.15);
            robot.lr_drive.setPower(.15);
            robot.rr_drive.setPower(.15);
            sleep(1200);

            robot.release.setPosition(.53);
            sleep(300);
            robot.lift.setTargetPosition(950);
            sleep(300);
            Backward(100,.4,90,25,60);
            robot.pivot_two.setPosition(leftservoin);
            robot.pivot_one.setPosition(rightservoin);
            robot.lift.setTargetPosition(0);
            StrafeRight(1150,.6,90,350,600);
            sleep(100);
            Backward(4475,.85,90,400,900);
            robot.intake.setPower(1);
            robot.intake_fold.setPosition(.387);
            sleep(600);
            Backward(75,.13,90,20, 50);
            sleep(400);
            Forward(220,.2,90,25,50);
            robot.intake_fold.setPosition(.5);
            sleep(400);
            robot.intake_fold.setPosition(.21);
            sleep(600);
            robot.intake_fold.setPosition(.5);
            sleep(400);
            robot.release.setPosition(.49);
            sleep(200);
            robot.intake.setPower(-1);
            Forward(4230,.85,90,400,1050);
            sleep(100);
            robot.intake.setPower(0);
            robot.pivot_two.setPosition(leftservoout);
            robot.pivot_one.setPosition(rightservoout);
            robot.lift.setTargetPosition(800);
            StrafeLeft(875,.6,90,300,500);
            sleep(100);

            robot.lf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.lf_drive.setPower(.15);
            robot.rf_drive.setPower(.15);
            robot.lr_drive.setPower(.15);
            robot.rr_drive.setPower(.15);
            sleep(1700);
            robot.release.setPosition(.53);
            sleep(200);
            robot.lift.setTargetPosition(1300);
            sleep(300);
            Backward(100,.3,90,25,30);
            robot.pivot_two.setPosition(leftservoin);
            robot.pivot_one.setPosition(rightservoin);
            robot.lift.setTargetPosition(0);
            sleep(1000);
        }

        //right
        if (markerpos == 3) {
            Backward(1060, .57, 0, 300, 700);
            sleep(100);
            TurnRight(90,.55,0,10,60);
            sleep(100);
            Backward(100, .3,90,20, 60);
            robot.intake.setPower(-1);
            sleep(500);
            robot.pivot_two.setPosition(leftservoout);
            robot.pivot_one.setPosition(rightservoout);
            robot.lift.setTargetPosition(375);
            robot.lift.setPower(1);
            Forward(1675,.6,90,350,700);
            robot.intake.setPower(0);
            sleep(100);
            StrafeRight(195,.35,90,30, 70);
            sleep(100);
            robot.lf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.lf_drive.setPower(.15);
            robot.rf_drive.setPower(.15);
            robot.lr_drive.setPower(.15);
            robot.rr_drive.setPower(.15);
            sleep(1200);

            robot.release.setPosition(.53);
            sleep(300);
            robot.lift.setTargetPosition(950);
            sleep(300);
            Backward(100,.4,90,25,60);
            robot.pivot_two.setPosition(leftservoin);
            robot.pivot_one.setPosition(rightservoin);
            robot.lift.setTargetPosition(0);
            StrafeRight(905,.6,90,300,600);
            sleep(100);
            Backward(4475,.85,90,400,900);
            robot.intake.setPower(1);
            robot.intake_fold.setPosition(.387);
            sleep(600);
            Backward(75,.13,90,20, 50);
            sleep(400);
            Forward(220,.2,90,25,50);
            robot.intake_fold.setPosition(.5);
            sleep(400);
            robot.intake_fold.setPosition(.21);
            sleep(600);
            robot.intake_fold.setPosition(.5);
            sleep(400);
            robot.release.setPosition(.49);
            sleep(200);
            robot.intake.setPower(-1);
            Forward(4230,.85,90,400,900);
            robot.pivot_two.setPosition(leftservoout);
            robot.pivot_one.setPosition(rightservoout);
            robot.lift.setTargetPosition(800);
            StrafeLeft(1250,.6,90,300,500);
            robot.intake.setPower(0);
            robot.lf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rf_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rr_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.lf_drive.setPower(.15);
            robot.rf_drive.setPower(.15);
            robot.lr_drive.setPower(.15);
            robot.rr_drive.setPower(.15);
            sleep(1700);
            robot.release.setPosition(.53);
            sleep(200);
            robot.lift.setTargetPosition(1300);
            sleep(300);
            Backward(100,.3,90,25,30);
            robot.pivot_two.setPosition(leftservoin);
            robot.pivot_one.setPosition(rightservoin);
            robot.lift.setTargetPosition(0);
            sleep(1000);
        }
    }
        public void TurnRight(double target, double inputspeed, double InitialHeading, double acc, double dec) {
        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;

        double startspeed = .2;
        double endspeed = .01;
        double speed = 0;
        double rightspeed = 0;
        double leftspeed = 0;
        double ediff = 0;
        double scalefactor = 525;
        double rightpower = 0;
        double leftpower = 0;

        //angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //heading = -angle.firstAngle;
            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        acc = InitialHeading + acc;
        dec = target - dec;

        while (((heading < (target - 1.5)) || (heading > (target + 1.5))) && opModeIsActive() && !isStopRequested()) {
            //angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            ediff = abs((robot.rf_drive.getCurrentPosition() + robot.rr_drive.getCurrentPosition()) / 2) - abs((robot.lf_drive.getCurrentPosition() + robot.lr_drive.getCurrentPosition()) / 2);

            if (heading < acc) {
                speed = startspeed + (heading / acc) * (inputspeed - startspeed);
            }
            if (heading > acc && heading <= dec) {
                speed = inputspeed;
            }
            if (heading > dec) {
                speed = endspeed + ((target - heading) / (target - dec)) * (inputspeed - endspeed) * .75;
            }

            rightpower = speed * (1 - (ediff / scalefactor));
            leftpower = speed * (1 + (ediff / scalefactor));

            robot.rf_drive.setPower(-rightpower);
            robot.rr_drive.setPower(-rightpower);
            robot.lf_drive.setPower(leftpower);
            robot.lr_drive.setPower(leftpower);
        }
        robot.rf_drive.setPower(0);
        robot.rr_drive.setPower(0);
        robot.lf_drive.setPower(0);
        robot.lr_drive.setPower(0);
    }

        public void TurnLeft(double target, double inputspeed, double InitialHeading, double acc, double dec) {
        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;
        double startspeed = .15;
        double endspeed = .01;
        double speed = 0;
        double rightspeed = 0;
        double leftspeed = 0;
        double ediff = 0;
        double scalefactor = 525;
        double rightpower = 0;
        double leftpower = 0;



        acc = InitialHeading + acc;
        dec = target - dec;
            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (((heading < (target - 1.5)) || (heading > (target + 1.5))) && opModeIsActive() && !isStopRequested()) {
            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            ediff = abs((robot.rf_drive.getCurrentPosition() + robot.rr_drive.getCurrentPosition()) / 2) - abs((robot.lf_drive.getCurrentPosition() + robot.lr_drive.getCurrentPosition()) / 2);

            if (heading > acc) {
                speed = startspeed + (heading / acc) * (inputspeed - startspeed);
            }
            if (heading <= acc && heading >= dec) {
                speed = inputspeed;
            }
            if (heading < dec) {
                speed = (endspeed + ((target - heading) / (target - dec)) * (inputspeed - endspeed)) * .75;
            }
            rightspeed = speed;
            leftspeed = -speed;

            rightpower = rightspeed * (1 - (ediff / scalefactor));
            leftpower = leftspeed * (1 + (ediff / scalefactor));

            robot.rf_drive.setPower(rightpower);
            robot.rr_drive.setPower(rightpower);
            robot.lf_drive.setPower(leftpower);
            robot.lr_drive.setPower(leftpower);
        }
        robot.rf_drive.setPower(0);
        robot.rr_drive.setPower(0);
        robot.lf_drive.setPower(0);
        robot.lr_drive.setPower(0);
    }

        public void Forward(int distance, double inputpower, double InitialHeading, int acc, int dec) {

        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;

        double startingspeed = .06;
        double rightspeed = 0;
        double leftspeed = 0;
        double currentleft = 0;
        double currentright = 0;
        double currentaverage = 0;
        double correctionfactor = 40;

            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        dec = distance - dec;

        while ((currentright < distance) && (currentleft < distance) && opModeIsActive() && !isStopRequested()) {
            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            currentright = abs(robot.rf_drive.getCurrentPosition());
            currentleft = abs(robot.lf_drive.getCurrentPosition());
            currentaverage = (currentright + currentleft) / 2;

            if (currentaverage <= acc) {
                rightspeed = (startingspeed + (currentaverage / acc) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / correctionfactor));
                leftspeed = (startingspeed + (currentaverage / acc) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / correctionfactor));
            }
            if (currentaverage > acc && currentaverage < dec) {
                rightspeed = inputpower * (1 + ((heading - InitialHeading) / correctionfactor));
                leftspeed = inputpower * (1 - ((heading - InitialHeading) / correctionfactor));
            }
            if (currentaverage >= dec) {
                rightspeed = (startingspeed + ((distance - currentaverage) / (distance - dec)) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / correctionfactor));
                leftspeed = (startingspeed + ((distance - currentaverage) / (distance - dec)) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / correctionfactor));
            }
            if (leftspeed > 1) {
                leftspeed = 1;
            }
            if (rightspeed > 1) {
                rightspeed = 1;
            }

            robot.lf_drive.setPower(leftspeed);
            robot.lr_drive.setPower(leftspeed);
            robot.rf_drive.setPower(rightspeed);
            robot.rr_drive.setPower(rightspeed);
        }
        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

        public void Backward(int distance, double inputpower, double InitialHeading, int acc, int dec) {

        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;

        double startingspeed = .1;
        double rightspeed = 0;
        double leftspeed = 0;
        double currentleft = 0;
        double currentright = 0;
        double currentaverage = 0;
        double correctionfactor = 40;

            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        dec = distance - dec;

        while ((currentright < distance) && (currentleft < distance) && opModeIsActive() && !isStopRequested()) {
            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            currentright = abs(robot.rf_drive.getCurrentPosition());
            currentleft = abs(robot.lf_drive.getCurrentPosition());
            currentaverage = (currentright + currentleft) / 2;

            if (currentaverage <= acc) {
                rightspeed = (startingspeed + (currentaverage / acc) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / correctionfactor));
                leftspeed = (startingspeed + (currentaverage / acc) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / correctionfactor));
            }
            if (currentaverage > acc && currentaverage < dec) {
                rightspeed = inputpower * (1 + ((heading - InitialHeading) / correctionfactor));
                leftspeed = inputpower * (1 - ((heading - InitialHeading) / correctionfactor));
            }
            if (currentaverage >= dec) {
                rightspeed = (startingspeed + ((distance - currentaverage) / (distance - dec)) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / correctionfactor));
                leftspeed = (startingspeed + ((distance - currentaverage) / (distance - dec)) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / correctionfactor));
            }
            if (leftspeed > 1) {
                leftspeed = 1;
            }
            if (rightspeed > 1) {
                rightspeed = 1;
            }

            robot.lf_drive.setPower(-rightspeed);
            robot.lr_drive.setPower(-rightspeed);
            robot.rf_drive.setPower(-leftspeed);
            robot.rr_drive.setPower(-leftspeed);
        }
        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void StrafeLeft(double distance, double inputpower, double InitialHeading, int acc, int dec) {

        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;

        double startingspeed = .15;
        double accdist = acc;
        double decdist = dec;
        double frontspeed = 0;
        double rearspeed = 0;
        double currentfront = 0;
        double currentrear = 0;
        double switchpoint = 0;
        double currentaverage = 0;
        double ediff = 0;
        double rightfront = 0;
        double leftfront = 0;
        double rightrear = 0;
        double leftrear = 0;
        double correctionfactor = 300;

        heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (((currentfront < distance) || (currentrear < distance)) && opModeIsActive() && !isStopRequested()) {
            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            rightfront = abs(robot.rf_drive.getCurrentPosition());
            leftfront = abs(robot.lf_drive.getCurrentPosition());
            rightrear = abs(robot.rr_drive.getCurrentPosition());
            leftrear = abs(robot.lr_drive.getCurrentPosition());
            currentfront = (rightfront + leftfront)/2;
            currentrear = (rightrear + leftrear)/2;
            currentaverage = (currentfront+currentrear)/2;
            ediff = (rightfront + leftrear)-(leftfront + rightrear);

            if (distance >= (accdist + decdist)) {
                if(currentaverage < accdist){
                    frontspeed = (startingspeed + (currentaverage / accdist) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / 40));
                    rearspeed = (startingspeed + (currentaverage / accdist) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / 40));
                }
                if (currentaverage >= accdist && (currentaverage < (distance - decdist))) {
                    frontspeed = inputpower * (1 + ((heading - InitialHeading) / 40));
                    rearspeed = inputpower * (1 - ((heading - InitialHeading) / 40));
                }
                if (currentaverage >= (distance - decdist)) {
                    frontspeed = (startingspeed + ((distance - currentaverage) / decdist) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / 40));
                    rearspeed = (startingspeed + ((distance - currentaverage) / decdist) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / 40));
                }
            }
            if (distance < (accdist + decdist)) {
                switchpoint = distance/3;
                if(currentaverage <= switchpoint){
                    frontspeed = (startingspeed + (currentaverage / switchpoint) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / 40));
                    rearspeed = (startingspeed + (currentaverage / switchpoint) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / 40));
                }
                if(currentaverage > switchpoint){
                    frontspeed = (startingspeed + ((distance - currentaverage) / (distance-switchpoint)) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / 40));
                    rearspeed = (startingspeed + ((distance - currentaverage) / (distance -switchpoint)) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / 40));
                }
            }
            robot.lf_drive.setPower(-frontspeed);
            robot.lr_drive.setPower(rearspeed);
            robot.rf_drive.setPower(frontspeed);
            robot.rr_drive.setPower(-rearspeed);
            telemetry.addData("ediff",ediff);
            telemetry.update();
        }
        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void StrafeRight(double distance, double inputpower, double InitialHeading,int acc, int dec) {

        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;

        double startingspeed = .15;
        double accdist = acc;
        double decdist = dec;
        double frontspeed = 0;
        double rearspeed = 0;
        double currentfront = 0;
        double currentrear = 0;
        double switchpoint = 0;
        double currentaverage = 0;
        double ediff = 0;
        double rightfront = 0;
        double leftfront = 0;
        double rightrear = 0;
        double leftrear = 0;
        double correctionfactor = 300;

        heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


        while (((currentfront < distance) || (currentrear < distance)) && opModeIsActive() && !isStopRequested()) {
            heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            rightfront = abs(robot.rf_drive.getCurrentPosition());
            leftfront = abs(robot.lf_drive.getCurrentPosition());
            rightrear = abs(robot.rr_drive.getCurrentPosition());
            leftrear = abs(robot.lr_drive.getCurrentPosition());
            currentfront = (rightfront + leftfront)/2;
            currentrear = (rightrear + leftrear)/2;
            currentaverage = (currentfront+currentrear)/2;
            ediff = (rightfront + leftrear)-(leftfront + rightrear);

            if (distance >= (accdist + decdist)) {
                if(currentaverage < accdist){
                    frontspeed = (startingspeed + (currentaverage / accdist) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / 40));
                    rearspeed = (startingspeed + (currentaverage / accdist) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / 40));
                }
                if (currentaverage >= accdist && (currentaverage < (distance - decdist))) {
                    frontspeed = inputpower * (1 + ((heading - InitialHeading) / 40));
                    rearspeed = inputpower * (1 - ((heading - InitialHeading) / 40));
                }
                if (currentaverage >= (distance - decdist)) {
                    frontspeed = (startingspeed + ((distance - currentaverage) / decdist) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / 40));
                    rearspeed = (startingspeed + ((distance - currentaverage) / decdist) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / 40));
                }
            }
            if (distance < (accdist + decdist)) {
                switchpoint = distance/3;
                if(currentaverage <= switchpoint){
                    frontspeed = (startingspeed + (currentaverage / switchpoint) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / 40));
                    rearspeed = (startingspeed + (currentaverage / switchpoint) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / 40));
                }
                if(currentaverage > switchpoint){
                    frontspeed = (startingspeed + ((distance - currentaverage) / (distance-switchpoint)) * (inputpower - startingspeed)) * (1 + ((heading - InitialHeading) / 40));
                    rearspeed = (startingspeed + ((distance - currentaverage) / (distance -switchpoint)) * (inputpower - startingspeed)) * (1 - ((heading - InitialHeading) / 40));
                }
            }
            robot.lf_drive.setPower(rearspeed);
            robot.lr_drive.setPower(-frontspeed);
            robot.rf_drive.setPower(-rearspeed);
            robot.rr_drive.setPower(frontspeed);
            telemetry.addData("ediff",ediff);
            telemetry.update();
        }
        robot.lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rr_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public static class MarkerDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum MarkerPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 125);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(150, 125);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(298, 125);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile marker_detection.MarkerDeterminationPipeline.MarkerPosition position = marker_detection.MarkerDeterminationPipeline.MarkerPosition.LEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                position = marker_detection.MarkerDeterminationPipeline.MarkerPosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                position = marker_detection.MarkerDeterminationPipeline.MarkerPosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = marker_detection.MarkerDeterminationPipeline.MarkerPosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public marker_detection.MarkerDeterminationPipeline.MarkerPosition getAnalysis()
        {
            return position;
        }
    }
}
