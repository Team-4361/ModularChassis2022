// package frc.Library.Autonomous;

// import java.util.HashMap;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonTrackedTarget;
// import org.photonvision.PhotonUtils;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;

// //Needs PhotonLib

// public class BallVisionCamera {

//     PhotonCamera photonCamera;

//     NetworkTableInstance inst;
//     NetworkTable rootTable;

//     double cameraHeight;
//     double cameraPitch;

//     //Height in meters of the yellow ball from Infinite Recharge
//     final double BALLHEIGHT = 0.1778;

//     /*
//       Our networktablename is photonvision
//       Our cameraName is RoxCam2021-4361
//       NetworkTable server is turned off
//     */
//     public BallVisionCamera(String networkTableHostName, String cameraName, double mCameraHeight, double mCameraPitch)
//     {
//         cameraHeight = mCameraHeight;

//         //camera pitch in radians
//         cameraPitch = mCameraPitch;

//         // inst = NetworkTableInstance.getDefault();

//         // inst.startClient(networkTableHostName);
//         // rootTable = inst.getTable("/photonvision/"+cameraName);

//         photonCamera = new PhotonCamera(cameraName/*rootTable*/);

//         photonCamera.setPipelineIndex(0);

//         System.out.println("Connected");
//     }

//     //Creates an image of the input video of the pipeline
//     public void takeInputPicture()
//     {
//         photonCamera.takeInputSnapshot();
//     }

//     //Creates an image of the output video of the pipeline
//     public void takeOutputSnapshot()
//     {
//         photonCamera.takeOutputSnapshot();
//     }

//     //Get the best target
//     public PhotonTrackedTarget getBestTarget()
//     {
//         return photonCamera.getLatestResult().getBestTarget();
//     }

//     //Gets the distance to the best target
//     public double getDistanceToTarget(PhotonTrackedTarget target)
//     {
//         return PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, BALLHEIGHT, Math.toRadians(cameraPitch), Math.toRadians(target.getPitch()));
//     }

//     public Boolean hasTargets()
//     {
//         return photonCamera.hasTargets();
//     }

//     // Returns the yaw--rotation around the vertical axis--in degrees
//     // 0 Yaw means the target is exactly in the middle of the screen
//     // Negative yaw means the target is somewhere on the left of the screen
//     // Positive yaw means the target is somewhere on the right of the screen
//     public double getYaw(PhotonTrackedTarget target)
//     {
//         return target.getYaw();
//     }

//     /*
//      Returns a hashmap of the required info needed to locate a ball and
//       move toward it
//     */
//     public HashMap<String, Double> getTargetGoal()
//     {
//         HashMap<String, Double> goalInfo = new HashMap<String, Double>();
//         PhotonTrackedTarget trackedTarget = getBestTarget();

//         goalInfo.put("Distance", getDistanceToTarget(trackedTarget));
//         goalInfo.put("Yaw", getYaw(trackedTarget));

//         return goalInfo;
//     }

// }
