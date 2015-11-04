/**
 * File:   UniversalDat.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class hold data which multiple classes must access.
 *
 * Last Modified 10/19/2015
 */


package com.alias.james.androidturtlebotui;

/**
 * Created by root on 10/2/15.
 */
public class UniversalDat
{
    enum CamPos
    {
        LEFT,
        RIGHT
    }

    /**
     * IP address of the robot (initialized to James' lab machine--internal network.
     * Alternate: 129.63.17.97 (James' lab machine--external network)
     */
    private static String ipAddressStr = "10.0.4.6";//TODO: change this for IP of robot brain 2
    private static String leftCamTopicStr = "/camera/rgb/image_raw"; /** !!!TEMP!!! ROS topic the left camera will publish to. */
    //private static String leftCamTopicStr = "/camera1/rgb/image_raw"; /** ROS topic the left camera will publish to. */
    private static String rightCamTopicStr = "/camera2/rgb/image_raw"; /** ROS topic the right camera will publish to. */
    private static int videoPort = 8080; /** The port which the video stream (VideoFrag.java) will use. */
    private static String camUrlStr = "http:"+ ipAddressStr + ":"+ videoPort +"/stream_viewer?topic=" + leftCamTopicStr; /** URL of the camera selected. */
    private static int leftCamPort = 50000; /** The port the left camera will use at class FetchLRFrames. */
    private static int rightCamPort = 50002; /** The port the right camera will use at class FetchLRFrames. */
    private static int datacomPort = 50001; /** The port which class DataCom will use. */




    /**
     * Mutator.
     * @see #ipAddressStr
     *
     * @param ipAddressStr
     */
    public static void setIpAddressStr(String ipAddressStr)
    {
        UniversalDat.ipAddressStr = ipAddressStr;
    }


    /**
     * Accessor.
     * @see #ipAddressStr
     *
     * @return
     */
    public static String getIpAddressStr()
    {
        return ipAddressStr;
    }


    /**
     * Mutator.
     * @see #leftCamTopicStr
     *
     * @param leftCamTopicStr
     */
    public static void setLeftCamTopicStr(String leftCamTopicStr)
    {
        UniversalDat.leftCamTopicStr = leftCamTopicStr;
    }


    /**
     * Accessor.
     * @see #leftCamTopicStr
     *
     * @return
     */
    public static String getLeftCamTopicStr()
    {
        return leftCamTopicStr;
    }


    /**
     * Mutator.
     * @see #rightCamTopicStr
     *
     * @param rightCamTopicStr
     */
    public static void setRightCamTopicStr(String rightCamTopicStr)
    {
        UniversalDat.rightCamTopicStr = rightCamTopicStr;
    }


    /**
     * Accessor.
     * @see #rightCamTopicStr
     *
     * @return
     */
    public static String getRightCamTopicStr()
    {
        return rightCamTopicStr;
    }


    /**
     * Mutator.
     * @see #camUrlStr
     *
     * @param camUrlStr
     */
    public static void setCamUrlStr(String camUrlStr)
    {
        UniversalDat.camUrlStr = camUrlStr;
    }


    /**
     * Accessor.
     * @see #camUrlStr
     *
     * @return
     */
    public static String getCamUrlStr()
    {
        return camUrlStr;
    }


    /**
     * Generates the URL by concatinating the given parameters with the static parts of the URL
     * (i.e. "http:", etc.).
     * @see #ipAddressStr
     *
     * @param ipAddressStr
     * @param camTopicStr
     * @param camPos
     */
    public static void catCamUrlStr(String ipAddressStr, String camTopicStr, CamPos camPos)
    {
        if(camPos == CamPos.LEFT)
        {
            setLeftCamTopicStr(camTopicStr);
        }
        else
        {
            setRightCamTopicStr(camTopicStr);
        }
        camUrlStr = "http:"+ ipAddressStr + ":8080/stream_viewer?topic=" + camTopicStr;
    }


    /**
     * Mutator.
     * @see #leftCamPort
     *
     * @param leftCamPort
     */
    public static void setLeftCamPort(int leftCamPort)
    {
        UniversalDat.leftCamPort = leftCamPort;
    }


    /**
     * Accessor.
     * @see #leftCamPort
     *
     * @return
     */
    public static int getLeftCamPort()
    {
        return leftCamPort;
    }


    /**
     * Mutator.
     * @see #rightCamPort
     *
     * @param rightCamPort
     */
    public static void setRightCamPort(int rightCamPort)
    {
        UniversalDat.rightCamPort = rightCamPort;
    }


    /**
     * Accessor.
     * @see #rightCamPort
     *
     * @return
     */
    public static int getRightCamPort()
    {
        return rightCamPort;
    }


    /**
     * Mutator.
     * @see #datacomPort
     *
     * @param datacomPort
     */
    public static void setDatacomPort(int datacomPort)
    {
        UniversalDat.datacomPort = datacomPort;
    }


    /**
     * Accessor.
     * @see #datacomPort
     *
     * @return
     */
    public static int getDatacomPort()
    {
        return datacomPort;
    }


    /**
     * Mutator.
     * @see #videoPort
     *
     * @param videoPort
     */
    public static void setVideoPort(int videoPort)
    {
        UniversalDat.videoPort = videoPort;
    }


    /**
     * Accessor.
     * @see #videoPort
     *
     * @return
     */
    public static int getVideoPort()
    {
        return videoPort;
    }


    /**
     * Prints out the current values of the class member variables.
     *
     * @return
     */
    public String toString()
    {
        String tmp = "----------------UniversalDat.toString()----------------" +
                "ipAddressStr: " + ipAddressStr + "\n" +
                "leftCamTopicStr: " + leftCamTopicStr + "\n" +
                "rightCamTopicStr: " + rightCamTopicStr + "\n" +
                "camUrlStr: " + camUrlStr + "\n" +
                "leftCamPort: " + leftCamPort + "\n" +
                "rightCamPort: " + rightCamPort + "\n" +
                "datacomPort: " + datacomPort +
                "videoPort: " + videoPort +
                "-------------------------------------------------------";
        return tmp;
    }

}// End of class UniversalDat