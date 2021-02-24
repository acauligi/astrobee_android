
/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

package edu.stanford.asl.geckoperchinggripper;

import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

import android.os.Handler;
import android.os.Looper;
import android.content.Intent;
import android.util.Log;

import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.topic.Publisher;

import sensor_msgs.JointState;
import std_msgs.String;

import java.net.URI;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.lang.Math;

import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.android.gs.StartGuestScienceService;
import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.PendingResult;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.Robot;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class StartGeckoperchinggripperService extends StartGuestScienceService {

    // The API implementation
    private ApiCommandImplementation api = null;

     /**
     * This function is called when the GS manager starts your apk.
     * Put all of your start up code in here.
     */
    NodeMainExecutor nodeMainExecutor;
    private GeckoGripperStatusNode gecko_gripper_node = null;

    /*
     * Handler and Runnable for permanent interface updating
     */
    public final long QUERY_WAIT_MS   = 1000;
    public final long STATUS_WAIT_MS  = 1000;
    public final long FILE_WAIT_MS    = 1500;
    public final long DL_WAIT_MS      = 1500;
    Handler handler;

    private Runnable queryRefresh = new Runnable() {
        @Override
        public void run() {
            gecko_gripper_node.sendQueryMsg();
        }
    };

    private Runnable allRefresh = new Runnable() {
        @Override
        public void run() {
            printAll();
        }
    };

    private Runnable statusRefresh = new Runnable() {
        @Override
        public void run() {
            printBinaryFlags();
        }
    };
    
    private Runnable fileRefresh = new Runnable() {
        @Override
        public void run() {
            printFileStatus();
        }
    };

    private Runnable delayRefresh = new Runnable() {
        @Override
        public void run() {
            printDelayStatus();
        }
    };

    // IP Address ROS Master and Hostname
    private static final URI ROS_MASTER_URI = URI.create("http://llp:11311");
    private static final java.lang.String ROS_HOSTNAME = "hlp";

    @Override
    public void onGuestScienceStart() {
        // Get a unique instance of the Astrobee API in order to command the robot.
        api = ApiCommandImplementation.getInstance();

        // Setting configurations for ROS-Android Node
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(ROS_HOSTNAME);
        nodeConfiguration.setMasterUri(ROS_MASTER_URI);

        gecko_gripper_node = new GeckoGripperStatusNode();

        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(gecko_gripper_node, nodeConfiguration);

        // Handler for interface updating
        this.handler = new Handler();

        // Inform the GS Manager and the GDS that the app has been started.
        sendStarted("info");
    }

    /**
     * This function is called when the GS manager stops your apk.
     * Put all of your clean up code in here. You should also call the terminate helper function
     * at the very end of this function.
     */
    @Override
    public void onGuestScienceStop() {
        // Stop the API
        api.shutdownFactory();

        handler.removeCallbacksAndMessages(null);

        // Inform the GS manager and the GDS that this app stopped.
        sendStopped("info");

        // Destroy all connection with the GS Manager.
        terminate();
    }

    /**
     * This function is called when the GS manager sends a custom command to your apk.
     * Please handle your commands in this function.
     *
     * @param command
     */
    @Override
    public void onGuestScienceCustomCmd(java.lang.String command) {
        /* Inform the Guest Science Manager (GSM) and the Ground Data System (GDS)
         * that this app received a command. */
        sendReceivedCustomCommand("info");

        try {
            // Transform the String command into a JSON object so we can read it.
            JSONObject jCommand = new JSONObject(command);

            // Get the name of the command we received. See commands.xml files in res folder.
            java.lang.String sCommand = jCommand.getString("name");

            // JSON object that will contain the data we will send back to the GSM and GDS
            JSONObject jResult = new JSONObject();

            switch (sCommand) {
                // You may handle your commands here
                case "gecko_gripper_open":
                    gecko_gripper_node.sendOpenGripper()
                    break;
                case "gecko_gripper_close":
                    gecko_gripper_node.sendCloseGripper()
                    break;
                case "gecko_gripper_engage":
                    gecko_gripper_node.sendGripperEngage()

                    handler.postDelayed(statusRefresh, STATUS_WAIT_MS);
                    break;
                case "gecko_gripper_disengage":
                    gecko_gripper_node.sendGripperDisengage()

                    handler.postDelayed(statusRefresh, STATUS_WAIT_MS);
                    break;
                case "gecko_gripper_lock":
                    gecko_gripper_node.sendGripperLock()

                    handler.postDelayed(statusRefresh, STATUS_WAIT_MS);
                    break;
                case "gecko_gripper_unlock":
                    gecko_gripper_node.sendGripperUnlock()

                    handler.postDelayed(statusRefresh, STATUS_WAIT_MS);
                    break;
                case "gecko_gripper_engage_lock":
                    gecko_gripper_node.sendGripperEngageLock()

                    handler.postDelayed(statusRefresh, STATUS_WAIT_MS);
                    break;
                case "gecko_gripper_enable_auto":
                    if (!gecko_gripper_node.feedbackPerchingEnable) {
                      gecko_gripper_node.sendEnableAuto();
                      handler.postDelayed(statusRefresh, STATUS_WAIT_MS);
                    }
                    break;
                case "gecko_gripper_disable_auto":
                    gecko_gripper_node.sendDisableAuto();

                    handler.postDelayed(statusRefresh, STATUS_WAIT_MS);
                    break;
                case "gecko_gripper_toggle_auto":
                    try {
                      JSONObject toggleAutoJson = new JSONObject();
                      toggleAutoJson.put("ERROR ", "toggle_auto not implemented!");
                      sendData(MessageType.JSON, "Command", toggleAutoJson.toString());
                    } catch (JSONException e) {
                        // Send an error message to the GSM and GDS
                        e.printStackTrace();
                        sendData(MessageType.JSON, "data", "ERROR parsing JSON");
                    }

                    break;
                case "gecko_gripper_mark_gripper":
                    gecko_gripper_node.sendMarkGripper(Double.parseDouble(jCommand.getString("IDX")));

                    handler.postDelayed(queryRefresh, QUERY_WAIT_MS);
                    handler.postDelayed(fileRefresh, FILE_WAIT_MS);

                    break;
                case "gecko_gripper_set_delay":
                    gecko_gripper_node.sendSetDelay(Double.parseDouble(jCommand.getString("DL_MS")));

                    handler.postDelayed(queryRefresh, QUERY_WAIT_MS);
                    handler.postDelayed(delayRefresh, DL_WAIT_MS);

                    break;
                case "gecko_gripper_close_exp":
                    gecko_gripper_node.sendCloseExp();

                    handler.postDelayed(queryRefresh, QUERY_WAIT_MS);
                    handler.postDelayed(fileRefresh, FILE_WAIT_MS);
                    break;
                case "gecko_gripper_status":
                    gecko_gripper_node.sendGripperStatus();
                    break;
                case "gecko_gripper_exp":
                    gecko_gripper_node.sendQueryIdx();
                    break;
                case "gecko_gripper_delay":
                    gecko_gripper_node.sendQueryDelay();
                    break;
                case "gecko_gripper_print_status":
                    gecko_gripper_node.sendQueryMsg();

                    handler.postDelayed(allRefresh, STATUS_WAIT_MS);

                    try {
                      JSONObject json = new JSONObject();
                      json.put("Summary", new JSONObject()
                          .put("Status", "PENDING")
                          .put("Message", "Sent gripper status query command"));
                      sendData(MessageType.JSON, "", json.toString());
                    } catch (JSONException e) {
                        // Send an error message to the GSM and GDS
                        e.printStackTrace();
                        sendData(MessageType.JSON, "data", "ERROR parsing JSON");
                    }
                    break;
                case "gecko_gripper_print_status_now":
                    printAll();
                    break;
                case "gecko_gripper_reset_gripper":
                    // After perching experiment, resets gripper
                    gecko_gripper_node.sendResetGripper();

                    handler.postDelayed(statusRefresh, STATUS_WAIT_MS);
                    break;
                case "gecko_gripper_enable_auto_feedback":
                    gecko_gripper_node.feedbackPerchingEnable = true;
                    gecko_gripper_node.sendResetGripper();

                    handler.postDelayed(statusRefresh, STATUS_WAIT_MS);
                    break;
                case "gecko_gripper_disable_auto_feedback":
                    gecko_gripper_node.feedbackPerchingEnable = false;
                    break;
                case "gecko_gripper_set_feedback_tol":
                    gecko_gripper_node.errorTol = 0.01*Double.parseDouble(jCommand.getString("TOL"));
                    gecko_gripper_node.feedbackPerchingEnable = false;

                    jResult.put("Feedback", "Feedback tolerance updated");
                    break;
                case "gecko_gripper_arm_deploy":
                    try {
                      JSONObject armDeployJson= new JSONObject();
                      Result armDeployResult = api.armDeploy();

                      if (armDeployResult.hasSucceeded()) {
                        armDeployJson.put("Status", "Arm deployment succeeded");
                      } else {
                        armDeployJson.put("Status", "Arm deployment failed");
                      }
                      sendData(MessageType.JSON, "Arm deploy ", armDeployJson.toString());
                    } catch (JSONException e) {
                        // Send an error message to the GSM and GDS
                        e.printStackTrace();
                        sendData(MessageType.JSON, "data", "ERROR parsing JSON");
                    }

                    break;
                case "gecko_gripper_arm_deploy_90":
                    try {
                      JSONObject armDeployJson= new JSONObject();
                      Result armDeployResult = api.armDeployNinety();

                      if (armDeployResult.hasSucceeded()) {
                        armDeployJson.put("Status", "Arm deployment succeeded");
                      } else {
                        armDeployJson.put("Status", "Arm deployment failed");
                      }
                      sendData(MessageType.JSON, "Arm deploy ", armDeployJson.toString());
                    } catch (JSONException e) {
                        // Send an error message to the GSM and GDS
                        e.printStackTrace();
                        sendData(MessageType.JSON, "data", "ERROR parsing JSON");
                    }

                    break;
                case "gecko_gripper_perch_pan_test":
                    boolean pan_test_succeeded = api.perchPanTest();
                    JSONObject perchPanResult = new JSONObject();
                    try {
                      if (pan_test_succeeded) {
                        perchPanResult.put("Status", "Perch pan test executed!");
                      } else {
                        perchPanResult.put("Status", "Perch pan test failed!");
                      }
                      sendData(MessageType.JSON, "Perch pan test ", perchPanResult.toString());
                    } catch (JSONException e) {
                        // Send an error message to the GSM and GDS
                        e.printStackTrace();
                        sendData(MessageType.JSON, "data", "ERROR parsing JSON");
                    }
                    break;
                case "gecko_gripper_perch_auto":
                    int timeout = 5; 
                    Kinematics currentKinematics = api.getTrustedRobotKinematics(timeout);
                    if (currentKinematics == null) {
                      // Kinematics can't be trusted
                      JSONObject kinFailureJson = new JSONObject();
                      sendData(MessageType.JSON, "EKF kinematics failed", kinFailureJson.toString());
                      return;
                    }

                    gecko_gripper_node.initPos = currentKinematics.getPosition();
                    gecko_gripper_node.initQuat = currentKinematics.getOrientation();

                    // Parse arguments sent by user
                    double offsetDistance = Double.parseDouble(jCommand.getString("DIST_CM")) / 100.0;
                    double DL_ = Double.parseDouble(jCommand.getString("DL_MS"));
                    double IDX_ = Double.parseDouble(jCommand.getString("IDX"));
                    java.lang.String axis = jCommand.getString("AXIS");

                    // Compute target position and orientation
                    gecko_gripper_node.targetQuat = gecko_gripper_node.initQuat;

                    if (axis.equals("px")) {
                      gecko_gripper_node.targetPos = new Point(
                        gecko_gripper_node.initPos.getX() + offsetDistance,
                        gecko_gripper_node.initPos.getY(),
                        gecko_gripper_node.initPos.getZ());
                    } else if (axis.equals("nx")) {
                      gecko_gripper_node.targetPos = new Point(
                        gecko_gripper_node.initPos.getX() - offsetDistance,
                        gecko_gripper_node.initPos.getY(),
                        gecko_gripper_node.initPos.getZ());
                    } else if (axis.equals("py")) {
                      gecko_gripper_node.targetPos = new Point(
                        gecko_gripper_node.initPos.getX(),
                        gecko_gripper_node.initPos.getY() + offsetDistance,
                        gecko_gripper_node.initPos.getZ());
                    } else if (axis.equals("ny")) {
                      gecko_gripper_node.targetPos = new Point(
                        gecko_gripper_node.initPos.getX(),
                        gecko_gripper_node.initPos.getY() - offsetDistance,
                        gecko_gripper_node.initPos.getZ());
                    } else if (axis.equals("pz")) {
                      gecko_gripper_node.targetPos = new Point(
                        gecko_gripper_node.initPos.getX(),
                        gecko_gripper_node.initPos.getY(),
                        gecko_gripper_node.initPos.getZ() + offsetDistance);
                    } else if (axis.equals("nz")) {
                      gecko_gripper_node.targetPos = new Point(
                        gecko_gripper_node.initPos.getX(),
                        gecko_gripper_node.initPos.getY(),
                        gecko_gripper_node.initPos.getZ() - offsetDistance);
                    } else {
                      JSONObject axisFailureJson = new JSONObject();
                      axisFailureJson.put("ERROR", "Invalid axis specified. Terminating");
                      sendData(MessageType.JSON, "Exiting", axisFailureJson.toString());
                      return;
                    }

                    // Send command to reset gripper
                    JSONObject resetGripperJson = new JSONObject();
                    resetGripperJson.put("Gripper Cmd", "Sending command to reset gripper");
                    sendData(MessageType.JSON, "Command", resetGripperJson.toString());
                    gecko_gripper_node.sendResetGripper();

                    // 500ms delay
                    long timeoutTime = System.currentTimeMillis() + 500;
                    while (System.currentTimeMillis() < timeoutTime) {
                      try {
                          Thread.sleep(10);
                      } catch (InterruptedException e) {
                          Log.e("LOG", "Runtime process was interrupted, terminating");
                          return;
                      }
                    }

                    // Do a check to make sure gripper has been reset
                    if (gecko_gripper_node.gripperState.getAdhesiveEngage() ||
                        gecko_gripper_node.gripperState.getWristLock() ||
                        gecko_gripper_node.gripperState.getAutomaticModeEnable()) {
                      JSONObject resetFailureJson = new JSONObject();
                      resetFailureJson.put("ERROR", "Failed to reset gripper");
                      sendData(MessageType.JSON, "Exiting", resetFailureJson.toString());
                      return;
                    } else { 
                      JSONObject resetSucceededJson = new JSONObject();
                      resetSucceededJson.put("Gripper Cmd", "Gripper has been successfully reset");
                      sendData(MessageType.JSON, "Status", resetSucceededJson.toString());
                    }
                    printBinaryFlags();

                    // Send message to user that delay is being sent
                    JSONObject setDelayJson = new JSONObject();
                    setDelayJson.put("Gripper Cmd", "Sending command to set delay")
                                .put("DL_MS", DL_);
                    sendData(MessageType.JSON, "Command", setDelayJson.toString());
                    gecko_gripper_node.sendSetDelay(DL_);

                    // 500ms delay
                    timeoutTime = System.currentTimeMillis() + 500;
                    while (System.currentTimeMillis() < timeoutTime) {
                      try {
                          Thread.sleep(10);
                      } catch (InterruptedException e) {
                          Log.e("LOG", "Runtime process was interrupted, terminating");
                          return;
                      }
                    }

                    gecko_gripper_node.sendQueryDelay();

                    // 500ms delay
                    timeoutTime = System.currentTimeMillis() + 500;
                    while (System.currentTimeMillis() < timeoutTime) {
                      try {
                          Thread.sleep(10);
                      } catch (InterruptedException e) {
                          Log.e("LOG", "Runtime process was interrupted, terminating");
                          return;
                      }
                    }

                    double reportedDL = (double)(gecko_gripper_node.gripperState.getDelay());
                    if (reportedDL != DL_) {
                      JSONObject delayFailureJson = new JSONObject();
                      delayFailureJson.put("ERROR", "Termianting with DL failure")
                                    .put("Desired DL_MS", DL_)
                                    .put("Reported DL_MS", reportedDL);
                      sendData(MessageType.JSON, "Exiting", delayFailureJson.toString());
                      return;
                    } else {
                      JSONObject dlSucceededJson = new JSONObject();
                      dlSucceededJson.put("Gripper Cmd", "Successfully set delay on gripper")
                                    .put("DL_MS", DL_);
                      sendData(MessageType.JSON, "Status", dlSucceededJson.toString());
                    }

                    printAll();

                    JSONObject markGripperJson = new JSONObject();
                    markGripperJson.put("Gripper Cmd", "Sending command to record SD file")
                                  .put("IDX", IDX_);
                    sendData(MessageType.JSON, "Command", markGripperJson.toString());
                    gecko_gripper_node.sendMarkGripper(IDX_);

                    // 500delay
                    timeoutTime = System.currentTimeMillis() + 500;
                    while (System.currentTimeMillis() < timeoutTime) {
                      try {
                          Thread.sleep(10);
                      } catch (InterruptedException e) {
                          Log.e("LOG", "Runtime process was interrupted, terminating");
                          gecko_gripper_node.sendCloseExp();
                          return;
                      }
                    }

                    gecko_gripper_node.sendQueryIdx();

                    // 500ms delay
                    timeoutTime = System.currentTimeMillis() + 500;
                    while (System.currentTimeMillis() < timeoutTime) {
                      try {
                          Thread.sleep(10);
                      } catch (InterruptedException e) {
                          Log.e("LOG", "Runtime process was interrupted, terminating");
                          gecko_gripper_node.sendCloseExp();
                          return;
                      }
                    }

                    double reportedIDX = (double)(gecko_gripper_node.gripperState.getExpIdx());
                    if (reportedIDX != IDX_) {
                      JSONObject idxFailureJson = new JSONObject();
                      idxFailureJson.put("ERROR", "Terminating with IDX failure")
                                    .put("Desired IDX", IDX_)
                                    .put("Reported IDX", reportedIDX);
                      sendData(MessageType.JSON, "Exiting", idxFailureJson.toString());
                      gecko_gripper_node.sendCloseExp();
                      return;
                    } else {
                      JSONObject markSucceededJson = new JSONObject();
                      markSucceededJson.put("Gripper Cmd", "Successfully opened file on SD")
                                      .put("IDX", IDX_);
                      sendData(MessageType.JSON, "Status", markSucceededJson.toString());
                    }
                      
                    JSONObject enableAutoJson = new JSONObject();
                    enableAutoJson.put("Gripper Cmd", "Sending command to enable auto");
                    sendData(MessageType.JSON, "Command", enableAutoJson.toString());
                    gecko_gripper_node.sendEnableAuto();

                    // 500ms delay
                    timeoutTime = System.currentTimeMillis() + 500;
                    while (System.currentTimeMillis() < timeoutTime) {
                      try {
                          Thread.sleep(10);
                      } catch (InterruptedException e) {
                          Log.e("LOG", "Runtime process was interrupted, terminating");
                          gecko_gripper_node.sendCloseExp();
                          return;
                      }
                    }

                    if (!gecko_gripper_node.gripperState.getAutomaticModeEnable()) { 
                      JSONObject enableAutoFailedJson = new JSONObject();
                      enableAutoFailedJson.put("ERROR", "Enable auto failed, terminating");
                      sendData(MessageType.JSON, "Exiting", enableAutoFailedJson.toString());
                      gecko_gripper_node.sendCloseExp();
                      return;
                    } else {
                      JSONObject enableAutoSucceededJson = new JSONObject();
                      enableAutoSucceededJson.put("Gripper Cmd", "Gripper now in automatic mode");
                      sendData(MessageType.JSON, "Status", enableAutoSucceededJson.toString());
                    }

                    printAll();

                    // Send data we are moving to pose
                    JSONObject perchStartJson = new JSONObject();
                    perchStartJson.put("Astrobee Cmd", "Starting perch maneuver!");
                    sendData(MessageType.JSON, "Command", perchStartJson.toString());

                    Result result = null;
                    result = api.moveTo(gecko_gripper_node.targetPos, gecko_gripper_node.targetQuat); 
                    if (!result.hasSucceeded()) {
                        // If planner does not report position tolerance violation, then perching was unsuccessful
                        java.lang.String resultMessage = result.getMessage();

                        Pattern pattern = Pattern.compile("Position tolerance violated", Pattern.CASE_INSENSITIVE);
                        Matcher matcher = pattern.matcher(resultMessage);
                        boolean matchFound = matcher.find();

                        if (!matchFound) {
                          // Planner has failed for some other reason i.e. perching has failed
                          JSONObject moveToJson = new JSONObject();
                          moveToJson.put("ERROR", "Planner failed unexpectedly!");
                          sendData(MessageType.JSON, "Exiting", moveToJson.toString());
                          gecko_gripper_node.sendCloseExp();
                          return;
                        }
                    } else{
                        // Planner has succeded, i.e. perching has failed
                        JSONObject moveToJson = new JSONObject();
                        moveToJson.put("ERROR", "Perching failed with recovery");
                        sendData(MessageType.JSON, "Exiting", moveToJson.toString());
                        gecko_gripper_node.sendCloseExp();
                        return;
                    }

                    currentKinematics = api.getTrustedRobotKinematics(timeout);
                    Point perchPos = currentKinematics.getPosition();

                    JSONObject perchPosJson = new JSONObject();
                    perchPosJson.put("Astrobee Cmd", "Successfully completed perch motion")
                                .put("Final Perch Position", perchPos.toString());
                    sendData(MessageType.JSON, "Status", perchPosJson.toString());

                    // Start timer to wait until adhesiveEngage and wristLock are both high or time out with failure
                    timeoutTime = System.currentTimeMillis() + 60000;
                    while (System.currentTimeMillis() < timeoutTime) {
                      try {
                          Thread.sleep(10);
                      } catch (InterruptedException e) {
                          Log.e("LOG", "Runtime process was interrupted, terminating");
                          gecko_gripper_node.sendCloseExp();
                          return;
                      }

                      if (gecko_gripper_node.gripperState.getAdhesiveEngage() && gecko_gripper_node.gripperState.getWristLock()) {
                          break;
                      }
                    }

                    if (!gecko_gripper_node.gripperState.getAdhesiveEngage() || !gecko_gripper_node.gripperState.getWristLock()) {
                      JSONObject perchFailedJson = new JSONObject();
                      perchFailedJson.put("ERROR", "Gripper did not perch. Terminating.")
                                      .put("Adhesive engage", gecko_gripper_node.gripperState.getAdhesiveEngage())
                                      .put("Wrist lock", gecko_gripper_node.gripperState.getWristLock());
                      sendData(MessageType.JSON, "Exiting", perchFailedJson.toString());
                      gecko_gripper_node.sendCloseExp();
                      return;
                    }

                    printAll();

                    result = api.moveTo(gecko_gripper_node.initPos, gecko_gripper_node.initQuat);
                    if (!result.hasSucceeded()) {
                        // If planner does not report position tolerance violation, then perching was unsuccessful
                        java.lang.String resultMessage = result.getMessage();
                        Pattern pattern = Pattern.compile("Position tolerance violated", Pattern.CASE_INSENSITIVE);
                        Matcher matcher = pattern.matcher(resultMessage);
                        boolean matchFound = matcher.find();

                        if (!matchFound) {
                          // Planner has failed for some other reason i.e. perching has failed
                          JSONObject moveToJson = new JSONObject();
                          moveToJson.put("ERROR", "Planner failed unexpectedly!");
                          sendData(MessageType.JSON, "Exiting", moveToJson.toString());
                          gecko_gripper_node.sendCloseExp();
                          return;
                        }
                    } else{
                        // Planner has succeded, i.e. perching has failed
                        JSONObject moveToJson = new JSONObject();
                        moveToJson.put("ERROR", "Perching failed with recovery");
                        sendData(MessageType.JSON, "Exiting", moveToJson.toString());
                        gecko_gripper_node.sendCloseExp();
                        return;
                    }

                    currentKinematics = api.getTrustedRobotKinematics(timeout);
                    Point postPerchPos = currentKinematics.getPosition();

                    // We do this in case the plan to move to the initial position fails for some other reason and not because perching succeeded
                    double postPerchDelta = Math.sqrt(Math.pow(postPerchPos.getX()-perchPos.getX(), 2) + 
                              Math.pow(postPerchPos.getY()-perchPos.getY(), 2) + 
                              Math.pow(postPerchPos.getZ()-perchPos.getZ(), 2));
                    if (postPerchDelta > 0.1) {
                      JSONObject graspFailureJson = new JSONObject();
                      graspFailureJson.put("ERROR", "Perch verification test failed. Terminating");
                      sendData(MessageType.JSON, "Exiting", graspFailureJson.toString());
                      gecko_gripper_node.sendCloseExp();
                      return;
                    } else {
                      JSONObject graspFailureJson = new JSONObject();
                      graspFailureJson.put("Gripper Status", "Successful perch validated");
                      sendData(MessageType.JSON, "Status", graspFailureJson.toString());
                    }

                    gecko_gripper_node.sendCloseExp();
                    timeoutTime = System.currentTimeMillis() + 500;
                    while (System.currentTimeMillis() < timeoutTime) {
                      try {
                          Thread.sleep(10);
                      } catch (InterruptedException e) {
                          Log.e("LOG", "Runtime process was interrupted, terminating");
                          gecko_gripper_node.sendCloseExp();
                          return;
                      }

                      if (!gecko_gripper_node.gripperState.getFileIsOpen()) {
                        break;
                      }
                    }
                    if (gecko_gripper_node.gripperState.getFileIsOpen()) {
                        JSONObject closeFileJson = new JSONObject();
                        closeFileJson.put("Gripper Status", "File on SD card still open! Make sure to close it");
                        sendData(MessageType.JSON, "Status", closeFileJson.toString());
                    }

                    JSONObject doneJson = new JSONObject();
                    doneJson.put("Astrobee Status", "Done with automatic perch sequence and exiting!");
                    sendData(MessageType.JSON, "Exiting", doneJson.toString());

                default:
                    // Inform GS Manager and GDS, then stop execution.
                    jResult.put("Summary", new JSONObject()
                        .put("Status", "ERROR")
                        .put("Message", "Unrecognized command"));
            }
        } catch (JSONException e) {
            // Send an error message to the GSM and GDS
            e.printStackTrace();
            sendData(MessageType.JSON, "data", "ERROR parsing JSON");
        } catch (Exception ex) {
            // Send an error message to the GSM and GDS
            ex.printStackTrace();
            sendData(MessageType.JSON, "data", "Unrecognized ERROR");
        }
    }

    public void printAll() {
      try{
        JSONObject jsonGripperState = gecko_gripper_node.gripperState.toJSON();
        sendData(MessageType.JSON, "Gripper status ", jsonGripperState.toString());
      } catch (JSONException e) {
          // Send an error message to the GSM and GDS
          e.printStackTrace();
          sendData(MessageType.JSON, "data", "ERROR parsing JSON");
      }
    }

    public void printBinaryFlags() {
      boolean adhesiveEngage = gecko_gripper_node.gripperState.getAdhesiveEngage();
      boolean wristLock = gecko_gripper_node.gripperState.getWristLock();
      boolean automaticModeEnable = gecko_gripper_node.gripperState.getAutomaticModeEnable();
      boolean fileIsOpen = gecko_gripper_node.gripperState.getFileIsOpen();

      try{
        JSONObject binaryStatus = new JSONObject();
        binaryStatus.put("Automatic Mode Enable", automaticModeEnable)
                    .put("Adhesive Engage", adhesiveEngage)
                    .put("Wrist Lock", wristLock)
                    .put("File is Open", fileIsOpen);
        sendData(MessageType.JSON, "Flags ", binaryStatus.toString());
      } catch (JSONException e) {
          // Send an error message to the GSM and GDS
          e.printStackTrace();
          sendData(MessageType.JSON, "data", "ERROR parsing JSON");
      }
    }

    public void printFileStatus() {
      boolean fileIsOpen = gecko_gripper_node.gripperState.getFileIsOpen();
      int expIdx = gecko_gripper_node.gripperState.getExpIdx();

      try{
        JSONObject json = new JSONObject();
        json.put("File is Open", fileIsOpen).put("Experiment Idx", expIdx);
        sendData(MessageType.JSON, "File status ", json.toString());
      } catch (JSONException e) {
          // Send an error message to the GSM and GDS
          e.printStackTrace();
          sendData(MessageType.JSON, "data", "ERROR parsing JSON");
      }
    }

    public void printDelayStatus() {
      int DL = gecko_gripper_node.gripperState.getDelay();

      try{
        JSONObject json = new JSONObject();
        json.put("Current delay in ms is ", DL);
        sendData(MessageType.JSON, "DL_MS ", json.toString());
      } catch (JSONException e) {
          // Send an error message to the GSM and GDS
          e.printStackTrace();
          sendData(MessageType.JSON, "data", "ERROR parsing JSON");
      }
    }
}
