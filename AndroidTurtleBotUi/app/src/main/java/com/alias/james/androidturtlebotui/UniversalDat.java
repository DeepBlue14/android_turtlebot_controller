/**
 * File:   UniversalDat.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class hold data which multiple classes must access.
 *
 * Last Modified 11/04/2015
 */


package com.alias.james.androidturtlebotui;

/**
 * Created by root on 10/2/15.
 */
public class UniversalDat
{

    /**
     * IP address of the robot (initialized to James' lab machine--internal network.
     * Alternate: 129.63.17.97 (James' lab machine--external network)
     */
    //private static String ipAddressStr = "10.0.4.6";
    //private static String ipAddressStr = "10.0.3.12";
    private static String ipAddressStr = "-1.-1.-1.-1";
    //private static int videoPort = 50000; /** The port for streaming the video. */
    private static int videoPort = -1; /** The port for streaming the video. */
    private static int positionOutPort = -1; /** The port which DataCom will use to send
                                                    coordinates to the robot. old default: 50001*/
    private static int positionInPort = -1; /** The position DataCom will use to receive
                                                   current coordinates from the robot. Old: 50002*/



    /**
     * Mutator
     * @see #ipAddressStr
     *
     * @param ipAddressStr
     * @return
     */
    public static void setIpAddressStr(String ipAddressStr)
    {
        UniversalDat.ipAddressStr = ipAddressStr;
    }


    public static String getIpAddressStr()
    {
        return ipAddressStr;
    }


    public static void setVideoPort(int videoPort)
    {
        UniversalDat.videoPort = videoPort;
    }


    public static int getVideoPort()
    {
        return videoPort;
    }


    public static void setPositionOutPort(int positionOutPort)
    {
        UniversalDat.positionOutPort = positionOutPort;
    }


    public static int getPositionOutPort()
    {
        return positionOutPort;
    }


    public static void setPositionInPort(int positionInPort)
    {
        UniversalDat.positionInPort = positionInPort;
    }


    public static int getPositionInPort()
    {
        return positionInPort;
    }


    /**
     * Prints out the current values of the class member variables.
     *
     * @return
     */
    public String toString()
    {
        String tmp = "----------------UniversalDat.toString()----------------" +
                "\nipAddressStr: " + getIpAddressStr() +
                "\nvideoPort: " + getVideoPort() +
                "\npositionOutPort: " + getPositionOutPort() +
                "\npositionInPort: " + getPositionInPort() +
                "\n-------------------------------------------------------";
        return tmp;
    }

}// End of class UniversalDat