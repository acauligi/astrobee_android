
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

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.message.MessageListener;
import org.ros.node.NodeConfiguration;
import org.ros.message.Time;
import android.util.Log;

import java.nio.*;
import java.util.*;
import java.util.Random;
import java.io.File;
import java.io.IOException;

import ff_msgs.ControlActionFeedback;
import sensor_msgs.JointState; 
import std_msgs.Header;
import std_msgs.String;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import edu.stanford.asl.geckoperchinggripper.types.GeckoGripperState;

public class GeckoGripperStatusNode extends AbstractNodeMain {

    public static GeckoGripperStatusNode instance = null;
    public static double errorPosition;
    public static boolean feedbackPerchingEnable = false;
    public static double errorTol = 0.18;

    public static Point initPos;
    public static Point targetPos;
    public static Quaternion initQuat;
    public static Quaternion targetQuat;

    GeckoGripperState gripperState;
    Publisher<JointState> mPublisher;
    Subscriber<JointState> mSubscriber;
    Subscriber<ControlActionFeedback> actionSubscriber;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("gecko_perching_gripper");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        // Creating subscribers for ROS Topics and adding message listeners
        mPublisher = connectedNode.newPublisher(
                "joint_goals",
                sensor_msgs.JointState._TYPE);

        mSubscriber = connectedNode.newSubscriber(
                "gecko_states", JointState._TYPE);
        mSubscriber.addMessageListener(new MessageListener<JointState>() {
            @Override
            public void onNewMessage(JointState jointState) {
                gripperState = updateGripperState(jointState);
                instance = GeckoGripperStatusNode.this;
            }
        }, 10);

        actionSubscriber = connectedNode.newSubscriber(
                "/gnc/control/feedback", ControlActionFeedback._TYPE);
        actionSubscriber.addMessageListener(new MessageListener<ControlActionFeedback>() {
            @Override
            public void onNewMessage(ControlActionFeedback feedback) {
                errorPosition = feedback.getFeedback().getErrorPosition();
                if (feedbackPerchingEnable && java.lang.Math.abs(errorPosition) >= errorTol) {
                  // TODO(acauligi): send command to engage & lock first time in this logic
                  // TODO(acauligi): add ability to configure 0.18cm tolerance (set_feedback_err)
                  sensor_msgs.JointState msg = mPublisher.newMessage();
                  java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
                  double[] msg_pos = new double[1];
                  msg_pos[0] = 0.;

                  msg_name.add("gecko_gripper_engage");
                  msg_name.add("gecko_gripper_lock");

                  msg.setName(msg_name);
                  msg.setPosition(msg_pos);
                  mPublisher.publish(msg);

                  feedbackPerchingEnable = false;
                  Log.i("LOG", "ControlActionFeedback: Cmd to engage & lock gripper sent");
                }
            }
        }, 10);

        instance = this;
    }

    public static GeckoGripperStatusNode getInstance() {
        return instance;
    }

    private GeckoGripperState updateGripperState(JointState jointState) {
        GeckoGripperState gripperState = new GeckoGripperState();

        // TODO(acauligi): determine how data is unpacked from jointState
        double[] position = jointState.getPosition();
        int gpg_n_bytes = 35;

        boolean readSD = (position[0] != 0);
        if (readSD) {
          // science packet
          // Android app doesn't support reading science data
          Log.i("LOG", "Received SD card data packet! Exiting");
          return gripperState;
        }

        int errorStatus = (int)position[1];
        int lastStatusReadTime = (int)position[2];
        boolean  overtemperatureFlag = (position[3] != 0);
        boolean experimentInProgress = (position[4] != 0);
        boolean fileIsOpen = (position[5] != 0);
        boolean automaticModeEnable = (position[6] != 0);
        boolean wristLock = (position[7] != 0);
        boolean adhesiveEngage = (position[8] != 0);
        int delay = (int)position[9];
        int expIdx = (int)position[10];

        if (overtemperatureFlag) {
          // overtemperatureFlag should always be true
          gripperState.setValidity(false);
          return gripperState;
        }

        if (errorStatus == 0) {
          // Only clear errorStatus
          gripperState.setErrorStatus(errorStatus);
        }
        gripperState.setLastStatusReadTime(lastStatusReadTime);
        gripperState.setOverTemperatureFlag(overtemperatureFlag);
        gripperState.setExperimentInProgress(experimentInProgress);
        gripperState.setFileIsOpen(fileIsOpen);
        gripperState.setAutomaticModeEnable(automaticModeEnable);
        gripperState.setWristLock(wristLock);
        gripperState.setAdhesiveEngage(adhesiveEngage);
        gripperState.setDelay(delay);
        gripperState.setExpIdx(expIdx);

        gripperState.setValidity(true);
        return gripperState;
    }

    public void sendSetDelay(double DL) {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_set_delay");
        msg_pos[0] = DL;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendQueryDelay() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_delay");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendMarkGripper(double IDX) {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_mark_gripper");
        msg_pos[0] = IDX;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }
    
    public void sendCloseExp() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_close_exp");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendQueryIdx() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_exp");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendQueryMsg() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[2];

        msg_name.add("gecko_gripper_delay");
        msg_name.add("gecko_gripper_exp");
        msg_pos[0] = 0.0;
        msg_pos[1] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendResetGripper() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();

        msg_name.add("gecko_gripper_disable_auto");
        msg_name.add("gecko_gripper_disengage");
        msg_name.add("gecko_gripper_unlock");
        msg_name.add("gecko_gripper_delay");
        msg_name.add("gecko_gripper_exp");

        double[] msg_pos = new double[5];
        msg_pos[0] = 0.0;
        msg_pos[1] = 0.0;
        msg_pos[2] = 0.0;
        msg_pos[3] = 0.0;
        msg_pos[4] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendEnableAuto() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_enable_auto");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendDisableAuto() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_disable_auto");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendOpenGripper() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_open");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendCloseGripper() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_close");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendGripperEngage() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_engage");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendGripperDisengage() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_disengage");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendGripperLock() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_lock");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendGripperUnlock() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];

        msg_name.add("gecko_gripper_unlock");
        msg_pos[0] = 0.0;

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendGripperEngageLock() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[2];
        msg_pos[0] = 0.;
        msg_pos[1] = 0.;

        msg_name.add("gecko_gripper_engage");
        msg_name.add("gecko_gripper_lock");

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }

    public void sendGripperStatus() {
        sensor_msgs.JointState msg = mPublisher.newMessage();
        java.util.List<java.lang.String> msg_name = new java.util.ArrayList<java.lang.String>();
        double[] msg_pos = new double[1];
        msg_pos[0] = 0.;

        msg_name.add("gecko_gripper_status");

        msg.setName(msg_name);
        msg.setPosition(msg_pos);
        mPublisher.publish(msg);
    }
}
