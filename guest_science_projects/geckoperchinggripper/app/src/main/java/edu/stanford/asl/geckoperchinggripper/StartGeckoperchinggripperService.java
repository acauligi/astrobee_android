
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

import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

import sensor_msgs.JointState;
import std_msgs.String;

import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.android.gs.StartGuestScienceService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class StartGeckoperchinggripperService extends StartGuestScienceService {

    // The API implementation
    // private ApiCommandImplementation api = null;

     /**
     * This function is called when the GS manager starts your apk.
     * Put all of your start up code in here.
     */
    private GeckoGripperStatusNode gecko_gripper_node;

    @Override
    public void onGuestScienceStart() {
        // Get a unique instance of the Astrobee API in order to command the robot.
        // api = ApiCommandImplementation.getInstance();

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
        // api.shutdownFactory();

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
            
            java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
            sensor_msgs.JointState msg = gecko_gripper_node.mPublisher.newMessage();

            switch (sCommand) {
                // You may handle your commands here
                case "gecko_gripper_open":
                    msg_name.add("perching_gecko_gripper_open");
                    // jResult.put("Summary", "perching_gecko_gripper_open");
                    break;
                case "gecko_gripper_close":
                    msg_name.add("perching_gecko_gripper_close");
                    // jResult.put("Summary", "perching_gecko_gripper_close");
                    break;
                case "gecko_gripper_engage":
                    msg_name.add("perching_gecko_gripper_engage");
                    // jResult.put("Summary", "perching_gecko_gripper_engage");
                    break;
                case "gecko_gripper_disengage":
                    msg_name.add("perching_gecko_gripper_disengage");
                    // jResult.put("Summary", "perching_gecko_gripper_disengage");
                    break;
                case "gecko_gripper_lock":
                    msg_name.add("perching_gecko_gripper_lock");
                    // jResult.put("Summary", "perching_gecko_gripper_lock");
                    break;
                case "gecko_gripper_unlock":
                    msg_name.add("perching_gecko_gripper_unlock");
                    // jResult.put("Summary", "perching_gecko_gripper_unlock");
                    break;
                case "gecko_gripper_enable_auto":
                    msg_name.add("perching_gecko_gripper_enable_auto");
                    // jResult.put("Summary", "perching_gecko_gripper_enable_auto");
                    break;
                case "gecko_gripper_disable_auto":
                    msg_name.add("perching_gecko_gripper_disable_auto");
                    // jResult.put("Summary", "perching_gecko_gripper_disable_auto");
                    break;
                case "gecko_gripper_toggle_auto":
                    // TODO(acauligi): return "cmd not implemented"
                    msg_name.add("perching_gecko_gripper_toggle_auto");
                    // jResult.put("Summary", "perching_gecko_gripper_toggle_auto");
                    break;
                case "gecko_gripper_mark_gripper":
                    // float IDX = (float) jCommand.get("IDX");
                    msg_name.add("perching_gecko_gripper_mark_gripper");
                    // jResult.put("Summary", "perching_gecko_gripper_mark_gripper");
                    break;
                case "gecko_gripper_set_delay":
                    // float DL = (float) jCommand.get("DL");
                    msg_name.add("perching_gecko_gripper_set_delay");
                    // jResult.put("Summary", "perching_gecko_gripper_set_delay");
                    break;
                case "gecko_gripper_open_exp":
                    // float IDX = (float) jCommand.get("IDX");
                    msg_name.add("perching_gecko_gripper_open_exp");
                    // jResult.put("Summary", "perching_gecko_gripper_open_exp");
                    break;
                case "gecko_gripper_next_record":
                    // float SKIP = (float) jCommand.get("SKIP");
                    msg_name.add("perching_gecko_gripper_next_record");
                    // jResult.put("Summary", "perching_gecko_gripper_next_record");
                    break;
                case "gecko_gripper_seek_record":
                    // float RN = (float) jCommand.get("RN");
                    msg_name.add("perching_gecko_gripper_seek_record");
                    // jResult.put("Summary", "perching_gecko_gripper_seek_record");
                    break;
                case "gecko_gripper_close_exp":
                    // float data = 0x00;
                    msg_name.add("perching_gecko_gripper_close_exp");
                    // jResult.put("Summary", "perching_gecko_gripper_close_exp");
                    break;
                case "gecko_gripper_status":
                    msg_name.add("perching_gecko_gripper_status");
                    // jResult.put("Summary", "perching_gecko_gripper_status");
                    break;
                case "gecko_gripper_record":
                    // float data = 0x00;
                    msg_name.add("perching_gecko_record");
                    // jResult.put("Summary", "perching_gecko_gripper_record");
                    break;
                case "gecko_gripper_exp":
                    // float data = 0x00;
                    msg_name.add("perching_gecko_gripper_exp");
                    // jResult.put("Summary", "perching_gecko_gripper_exp");
                    break;
                case "gecko_gripper_delay":
                    // float data = 0x00;
                    msg_name.add("perching_gecko_gripper_delay");
                    // jResult.put("Summary", "perching_gecko_gripper_delay");
                    break;

                default:
                    // Inform GS Manager and GDS, then stop execution.
                    jResult.put("Summary", new JSONObject()
                        .put("Status", "ERROR")
                        .put("Message", "Unrecognized command"));
            }

            msg.setName(msg_name);
            gecko_gripper_node.mPublisher.publish(msg);

            // Send data to the GS manager to be shown on the Ground Data System.
            sendData(MessageType.JSON, "data", jResult.toString());
        } catch (JSONException e) {
            // Send an error message to the GSM and GDS
            sendData(MessageType.JSON, "data", "ERROR parsing JSON");
        } catch (Exception ex) {
            // Send an error message to the GSM and GDS
            sendData(MessageType.JSON, "data", "Unrecognized ERROR");
        }
    }
}