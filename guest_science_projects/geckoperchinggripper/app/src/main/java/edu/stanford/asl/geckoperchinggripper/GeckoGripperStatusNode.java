
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
import org.ros.message.Time;
import android.util.Log;
import java.nio.ByteBuffer;

import sensor_msgs.JointState; 
import std_msgs.Header;
import std_msgs.String;

import edu.stanford.asl.geckoperchinggripper.types.GeckoGripperState;


public class GeckoGripperStatusNode extends AbstractNodeMain {

    public static GeckoGripperStatusNode instance = null;

    GeckoGripperState gripperState;
    Subscriber<JointState> mSubscriber;
    Publisher<JointState> mPublisher;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("battery_status_monitor");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        // Creating subscribers for ROS Topics and adding message listeners
        mSubscriber = connectedNode.newSubscriber(
                "joint_states", sensor_msgs.JointState._TYPE);
        mSubscriber.addMessageListener(new MessageListener<JointState>() {
            @Override
            public void onNewMessage(JointState jointState) {
                gripperState = updateGripperState(jointState);
                instance = GeckoGripperStatusNode.this;
                Log.i("LOG", "MSG TOP LEFT");
            }
        }, 10);

        mPublisher = connectedNode.newPublisher(
                "joint_goals",
                sensor_msgs.JointState._TYPE);

        instance = this;
    }

    public static GeckoGripperStatusNode getInstance() {
        return instance;
    }

    /*
    public void sendMessage(String cmd, float data) {
        if (mPublisher == null) {
            return;
        }

        sensor_msgs.JointState gpg_cmd = mPublisher.newMessage();
        // std_msgs.Header hdr = mMsgFac.newFromType(Header._TYPE);
        Time myTime = new Time();

        // TODO(acauligi): what should go here?
        myTime.secs = 1487370000;
        myTime.nsecs = 0;
        gpg_cmd.getHeader().setStamp(myTime);
        // gpg_cmd.setHeader(hdr);

        String prefix_ = "top_aft_";

        // set first value of name
        java.util.List<std_msgs.String> nList
            = new java.util.ArrayList<std_msgs.String>();

        std_msgs.String new_str_msg = std_msgs.String();
        new_str_msg.setData(prefix_ + cmd);
        nList.add(new_str_msg);

        gpg_cmd.setName(nList);

        // set first value of position
        java.util.List<std_msgs.Float64> pList
            = new java.util.ArrayList<std_msgs.Float64>();
        pList.add(data);
        gpg_cmd.setPosition(pList);

        mPublisher.publish(gpg_cmd);
    }
    */

    /**
     * Method that updates the information about the gecko perching gripp
     *
     * @param jointState Java Object for sensor_msg/JointState ROS message
     * @return
     */
    private GeckoGripperState updateGripperState(JointState jointState) {
        GeckoGripperState gripperState = new GeckoGripperState();

        // TODO(acauligi): determine how data is unpacked from jointState
        double[] position = jointState.getPosition();
        int gpg_n_bytes = 6;
        int ii = 0;

        byte[] byte_data;
        byte_data = ByteBuffer.allocate(8).putDouble(position[7+ii]).array();

        if (byte_data[0] != 0xFF || byte_data[1] != 0xFF || byte_data[2] != 0xFD) {
          // Bad data
        }

        byte ID = byte_data[3]; // 0x00 for status packet, 0x01 for science packet
        byte ERR_L = byte_data[4];
        byte ERR_H = byte_data[5];
        byte TIME_L = byte_data[6];
        byte TIME_H = byte_data[7];

        boolean overtemperature_flag = false;
        boolean experiment_in_progress = false;
        boolean file_is_open = false;
        boolean automatic_mode_enable = false;
        boolean wrist_lock = false;
        boolean adhesive_engage = false;
        if (ID == 0x00) {
          // status packet
          ii++;
          byte_data = ByteBuffer.allocate(8).putDouble(position[7+ii]).array();
          byte STATUS_H = byte_data[0];
          byte STATUS_L = byte_data[1];

          int mask = 0x80;
          overtemperature_flag = (STATUS_H & mask) != 0;

          mask = 0x01;
          experiment_in_progress = (STATUS_H & mask) != 0;

          mask = 0x40;
          file_is_open = (STATUS_L & mask) != 0;

          mask = 0x08;
          automatic_mode_enable = (STATUS_L & mask) != 0;

          mask = 0x02;
          wrist_lock = (STATUS_L & mask) != 0;

          mask = 0x01;
          adhesive_engage = (STATUS_L & mask) != 0;
        } else {
          // TODO
        }

        // gripperState.setLastStatusReadTime(jointState.getSerialNumber());
        // gripperState.setErrorStatus();
        // gripperState.setAdhesiveEngage();
        // gripperState.setWristLock();
        // gripperState.setAutomaticModeEnable();
        // gripperState.setExperimentInProgress();
        // gripperState.setOverTemperatureFlag();
        // gripperState.setFileIsOpen();
        // gripperState.setExpIdx();
        // gripperState.setDelay();
        return gripperState;

        // guestsciencedata datamsg = mmessagefactory.newfromtype(guestsciencedata._type);
        // header hdr = mmessagefactory.newfromtype(header._type);

        // hdr.setstamp(mnodeconfig.gettimeprovider().getcurrenttime());
        // datamsg.setheader(hdr);

        // datamsg.setapkname(apkfullname);

        // if (msg.what == messagetype.string.toint()) {
        //     datamsg.setdatatype(guestsciencedata.string);
        // } else if (msg.what == messagetype.json.toint()) {
        //     datamsg.setdatatype(guestsciencedata.json);
        // } else if (msg.what == messagetype.binary.toint()) {
        //     datamsg.setdatatype(guestsciencedata.binary);
        // } else {
        //     mlogger.error(log_tag, "message type in guest science message is unknown so the message " +
        //             "will not be sent to the ground.");
        //     return;
        // }

        // datamsg.settopic(topic);

        // // if there isn't data, don't copy it over as it will crash
        // if (data.length != 0) {
        //     channelbuffer databuff = channelbuffers.wrappedbuffer(byteorder.little_endian, data);
        //     datamsg.setdata(databuff);
        // }
        // mdatapublisher.publish(datamsg);
    }
}