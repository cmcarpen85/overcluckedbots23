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

@Autonomous(name = "camera_test", group = "Blue")
public class CameraTest extends LinearOpMode {
  MarkerDeterminationPipeline pipeline;
  OpenCvWebcam webcam;

  IMU imu = null;

  boolean done = false;

  String markerpos = "Center";

  @Override
  public void runOpMode() {
    int cameraMonitorViewId = hardwareMap
      .appContext.getResources()
      .getIdentifier(
        "cameraMonitorViewId",
        "id",
        hardwareMap.appContext.getPackageName()
      );
    webcam =
      OpenCvCameraFactory
        .getInstance()
        .createWebcam(
          hardwareMap.get(WebcamName.class, "Webcam 1"),
          cameraMonitorViewId
        );
    pipeline = new MarkerDeterminationPipeline();
    webcam.setPipeline(pipeline);
    webcam.openCameraDeviceAsync(
      new OpenCvCamera.AsyncCameraOpenListener() {

        @Override
        public void onOpened() {
          webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError(int errorCode) {
          /*
           * This will be called if the camera could not be opened
           */
        }
      }
    );

    waitForStart();
    //Start of Code
    
    int count = 0;
    do {
      if (
        pipeline.getAnalysis() == MarkerDeterminationPipeline.MarkerPosition.RIGHT
      ) {
        markerpos = "Right";
      }
      else if (
        pipeline.getAnalysis() ==
        MarkerDeterminationPipeline.MarkerPosition.CENTER
      ) {
        markerpos = "Center";
      }
      else if (
        pipeline.getAnalysis() == MarkerDeterminationPipeline.MarkerPosition.LEFT
      ) {
        markerpos = "Left";
      }

      telemetry.addData("Position", markerpos);
      telemetry.update();

      sleep(100);
      count++;
    } while (count < 1000);
    
    sleep(30000);
  }
}
