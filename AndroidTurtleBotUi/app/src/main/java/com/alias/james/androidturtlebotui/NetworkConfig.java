/*
 * File:   NetworkConfig.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class allows an user/administrator to enter the ip address, port, and
 *                   ROS topics that the program should be listening to.  It also will display
 *                   a list of the computers available on the network.
 *
 * References: See AndroidVideoIp2 mainActivity.java
 *             http://stackoverflow.com/questions/2586301/set-inputtype-for-an-edittext
 *             http://stackoverflow.com/questions/2784081/android-create-spinner-programmatically-from-array
 *
 * <p>
 *             (for backend)
 *             nmap -sP 10.0.4.1/24 #displays all computers on the network
 *             nmap -PN 10.0.4.6 #displays open ports on the specified computer
 *             https://www.digitalocean.com/community/tutorials/how-to-use-nmap-to-scan-for-open-ports-on-your-vps
 *             http://stackoverflow.com/questions/478898/how-to-execute-a-command-and-get-output-of-command-within-c
 * </p>
 *
 * Last Modified: 8/11/2015
 */

package com.alias.james.androidturtlebotui;


import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.EditText;

public class NetworkConfig
{
    //private String ipAddressStr = "10.0.4.6"; /** Is the IP address of the robot.  Currently hard-coded to James K's lab machine robot-lab6. */
    //private String ipAddressStr = "129.63.17.97";
    //private int portInt = 8080; /** Is the port that the video stream will be sent to. */
    //private String leftCamTopic = "/usb_cam1/image_raw"; /** Is the ROS topic that the left camera will stream to. */
    //private String rightCamTopic = "/usb_cam2/image_raw"; /** Is the ROS topic that the right camera will stream to. */
    //private String camUrlStr = "http:10.0.4.6:8080/stream_viewer?topic=/camera/rgb/image_rect_color"; /** Is the HTTP address the robot will stream video to. */
    //private String camUrlStr = "http:129.63.17.97:8080/stream_viewer?topic=/camera/rgb/image_rect_color";
    private AlertDialog.Builder networkDialog; /** Is the dialog which will present networking options (if the user is logged in as administrator. */


    /**
     * Initializes the network configuration dialog (which can only be accessed by
     * an administrator).
     *
     * @param activity
     * @param layoutInflater
     */
    public void initNetworkConfigDialog(Activity activity, LayoutInflater layoutInflater)
    {
        LayoutInflater inflater = layoutInflater;
        final View view = inflater.inflate(R.layout.network_config, null);

        networkDialog = new AlertDialog.Builder(activity)
                .setTitle("Frankenscooter UI")
                .setMessage("Welcome to Frankenscooter UI")
                .setView(view)
                .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        EditText ipAddressUi = (EditText) view.findViewById(R.id.ip_address);
                        EditText port1Ui = (EditText) view.findViewById(R.id.port_1);
                        EditText port2Ui = (EditText) view.findViewById(R.id.port_2);
                        EditText port3Ui = (EditText) view.findViewById(R.id.port_3);

                        if (Login.getUserIsAdmin()) {
                            System.out.println("^^^Admin; read new data");
                            if (!ipAddressUi.getText().toString().isEmpty()) {
                                UniversalDat.setIpAddressStr(ipAddressUi.getText().toString());
                            }
                            if (!port1Ui.getText().toString().isEmpty()) {
                                UniversalDat.setVideoPort(Integer.parseInt(port1Ui.getText().toString()));
                            }
                            if (!port2Ui.getText().toString().isEmpty()) {
                                UniversalDat.setPositionOutPort(Integer.parseInt(port2Ui.getText().toString()));
                            }
                            if (!port3Ui.getText().toString().isEmpty()) {
                                UniversalDat.setPositionInPort(Integer.parseInt(port2Ui.getText().toString()));
                            }

                        } else {
                            System.out.println("^^^NOT admin; reject data");
                        }
                        //setUrl(ipAddressStr, portInt);
                    }
                })
                .setNegativeButton(android.R.string.cancel, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        System.out.println("\"Cancel\" alert button selected");
                        //System.exit(0);
                    }
                })
                .setIcon(android.R.drawable.ic_dialog_alert);
    }


    /**
     * Mutator.
     * @see #networkDialog
     *
     * @param networkDialog
     */
    public void setNetworkDialog(AlertDialog.Builder networkDialog)
    {
        this.networkDialog = networkDialog;
    }


    /**
     * Accessor.
     * @see #networkDialog
     *
     * @return
     */
    public AlertDialog.Builder getNetworkDialog()
    {
        return networkDialog;
    }


    /**
     * TODO: implement this.
     *
     * @return
     */
    public String toString()
    {
        return "^^^*** METHOD STUB ***^^^";
    }


} // End of class NetworkConfig