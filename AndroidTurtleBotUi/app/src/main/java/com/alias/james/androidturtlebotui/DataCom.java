/*
 * File:   DataCom.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class handles data communication between the app and the robot.  It
 *                   sends commands to the robot, and receives updates as to the robots current
 *                   position.
 *
 * Commands:
 *      - A list of values representing a path between the current position and the final location.
 *      - An int (enum of the app side) to represent a simple command (i.e.  say "hello world").
 *
 * Notes: rename file/class to VideoClient
 *
 * Last Modified 10/19/2015
 */


package com.alias.james.androidturtlebotui;

import android.app.Activity;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.AsyncTask;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import android.view.LayoutInflater;

import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class DataCom
{
    private Socket socket; /** Socket object to handle traffic between robot and Android device. */
    private String m_msg; /** Contains the (x, y) coordinates to be sent to the robot for evaluation as a string sequence: "|x|y|". */
    private String yOrNMsg; /** Message sent to the robot to confirm or deny that the object with the box drawn around it was the correct one (will be Y or N). */
    //private static final int SERVER_PORT = 50001; /** Is the port that DataCom will use by default. */
    //private String ipAddressStr = "10.0.4.6"; /** IP Address of robot.  Currently hard-coded to James's lab machine (robot-lab6) */
    //private String ipAddressStr = "129.63.17.97"; /** IP Address of robot.  Currently hard-coded to James's lab machine (robot-lab6) */
    private final int MATRIX_SIZE = 921600; /** OpenCV matrixes  retrieved from the PrimeSense via ROS are always this size */
    private static Bitmap bBoxBitmap; /** Is the bitmap with bounding box drawn around the object the robot thinks the user selected. */
    private Activity activity; /** A reference to the UI's Activity object (to be used to inflate an alert dialog. */
    private LayoutInflater inflater; /** A reference to the UI's LayoutInflater object. */
    private InetAddress serverAddress = null; /** The address of the robot. */
    private DataCom dataCom = this; /** Is an object to self. */


    /**
     * This constructor method loads OpenCV.
     */
    public DataCom() {
        System.out.println("^^^Starting DataCom...^^^");

        if(!OpenCVLoader.initDebug())
        {
            System.err.println("^^^Failed to load OpenCV @ DataCom::DataCom()");
        }

    }


    /**
     * Sets the UI member variables.
     *
     * @param activity
     * @param inflater
     */
    public void setUiStuff(Activity activity, LayoutInflater inflater) {
        this.activity = activity;
        this.inflater = inflater;
    }


    /**
     * Accessor to create an return a new inner class Fetch object.
     *
     * @return new Fetch(), a new Fetch() object
     */
    public Fetch genFetch() {
        return new Fetch();
    }


    protected class Fetch extends AsyncTask<Integer, Integer, Long>
    {

        /**
         * Mutator.
         * @see #m_msg
         *
         * @param msg
         */
        public void setMsg(String msg) {
            System.out.println("^^^Setting Msg");
            m_msg = msg;
        }


        /**
         * Accessor.
         * @see #m_msg
         *
         * @return
         */
        public String getMsg() {
            return m_msg;
        }


        /**
         * Sends the message to the robot via the socket.
         * @see #socket
         *
         * @param msg
         */
        public void sendMsg(String msg) {
            try {
                System.out.println("^^^Sending message...");
                PrintWriter out = new PrintWriter(new BufferedWriter(new OutputStreamWriter(socket.getOutputStream())), true); // ???Do this once???
                out.println(msg);
                System.out.println("^^^Message has been sent");
            } catch (UnknownHostException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }


        /**
         * Connects the Android device (client) to the robot (server).
         */
        private void connect() {
            System.out.println("^^^@ connect()");

            try {
                serverAddress = InetAddress.getByName(UniversalDat.getIpAddressStr() );
                socket = new Socket(serverAddress, UniversalDat.getDatacomPort() ); //!!!connect in a separate method--same goes for FetchLRFrames!!!
                System.out.println("^^^Successfully connected to server^^^");
            } catch (UnknownHostException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }

        }


        /**
         * Checks to see of a connection to the server has already been made.  If it has not been
         * done, it calls connect().  It then sends the pixel coordinates to the robot, and
         * receives a OpenCV matrix.
         *
         * @param params
         * @return
         */
        @Override
        protected Long doInBackground(Integer... params)
        {
            try
            {
                if(serverAddress == null)
                {
                    System.out.println("^^^Connecting");
                    connect();
                }

                sendMsg(getMsg());

                InputStream sub = socket.getInputStream(); // ???Do this once???
                System.out.println("^^^Generating new InputStream obj");
                // !!!???will this be true 32 & 64 bit processors???!!!
                byte[] buffer = new byte[MATRIX_SIZE];
                int currPos = 0;
                int bytesRead = 0;

                //bytesRead = sub.read(buffer, 0, buffer.length);
                //currPos = bytesRead;
                do
                {
                    bytesRead = sub.read(buffer, currPos, (buffer.length - currPos));

                    if (bytesRead != -1 && bytesRead != 0)
                    {
                        currPos += bytesRead;
                    }
                    else
                    {
                        break;
                    }

                } while (bytesRead != -1 && bytesRead != 0);

                System.out.println("^^^recieved image bytes @ DataCom::doInBackground(...):" + currPos);
                Mat mat = new Mat(480, 640, CvType.CV_8UC3);
                mat.put(0, 0, buffer);
                bBoxBitmap = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(mat, bBoxBitmap);
            } catch (UnknownHostException e) {
                System.out.println("^^^Ousted");
                e.printStackTrace();
            } catch (IOException e) {
                System.out.println("^^^Ousted");
                e.printStackTrace();
            }


            return null;
        }


        /**
         * This method is implicitly called after doInBackground(Integer...) finishes executing.
         * @see #doInBackground(Integer...)
         * It adds creates a custom AlertDialog to display the bitmap containing the object with
         * a bounding box, and prompts the user to choose whether the object was the correct
         * one.
         *
         * @param aLong
         */
        @Override
        protected void onPostExecute(Long aLong) {
            super.onPostExecute(aLong);

        }

        /**
         * Closes the socket.
         */
        public void close() {
            try {
                socket.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }// End of inner class


    /**
     * Accesssor.
     * @see #bBoxBitmap
     *
     * @return
     */
    public static Bitmap getlFrame() {
        return bBoxBitmap;
    }

}// End of outer class DataCom