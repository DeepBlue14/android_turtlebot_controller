/*
 * File:   CamClient.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class contains a thread which will receive the video feed from the robot.
 *
 *
 * Reference: http://developer.android.com/intl/ko/training/multiple-threads/communicate-ui.html
 * // opencv stuff: http://stackoverflow.com/questions/17767557/how-to-use-opencv-in-android-studio-using-gradle-build-tool
 *
 *
 * Last Modified 11/08/2015
 */


package com.alias.james.androidturtlebotui;

import android.app.Activity;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.os.Handler;
import android.widget.ImageView;

import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.IOException;
import java.io.InputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.text.BreakIterator;

/**
 * Created by root on 11/9/15.
 */
public class CamClient /*extends Activity */implements Runnable {

    //TODO: main() should notify this thread which fragment is loaded
    enum CurrentFrag {
        MAP,
        CAM,
        TELEOP
    }

    private static CurrentFrag currentFrag;
    private int count=0;
    private ImageView imageView;
    private Bitmap bitmap;
    private InetAddress serverAddress = null;
    private Socket socket;
    private final int MATRIX_SIZE = 921600;
    private boolean isConnected = false;
    private boolean isIdling = false;
    Handler handler;

    public CamClient() {

        if(!OpenCVLoader.initDebug() ) {
            System.err.println("^^^Failed to load OpenCV @ FetchLRFrames::FetchLRFrames()");
        }
        handler = new Handler();
    }


    @Override
    public void run() {

        byte[] buffer = new byte[MATRIX_SIZE];
        int currPos = 0;
        int bytesRead = 0;
        InputStream sub;
        Mat mat = new Mat(480, 640, CvType.CV_8UC3);

        while (true) {
            if(imageView == null)
            {
                //imageView = MapFrag.getCamImageView();
                imageView = JoyFrag.getViewImageView();
            }
            if (!isIdling && imageView != null) {

                if (!isConnected) {
                    connect();
                }

                try {
                    if (socket != null) {
                        sub = socket.getInputStream();
                    } else {
                        sub = null; //FIXME: hack
                    }

                    if (sub != null) {

                        currPos = 0;
                        bytesRead = 0;

                        do {
                            bytesRead = sub.read(buffer, currPos, (buffer.length - currPos));
                            currPos += bytesRead;
                        } while (bytesRead != -1 && bytesRead != 0);

                        mat.put(0, 0, buffer);

                        if (currPos == 921600) {
                            bitmap = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.RGB_565);
                            Utils.matToBitmap(mat, bitmap);

                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    imageView.setImageBitmap(bitmap);
                                }
                            });

                        }
                    }

                } catch (IOException e) {
                    System.out.println("^^^kicked out :(");
                    e.printStackTrace();
                }
            } else {
                ;//spin (i.e. have the thread idle)
            }

        } // End of outer loop
    }


    public void connect() {
        System.out.println("^^^@ connect()");
        try {
            //serverAddress = InetAddress.getByName("10.0.4.6");
            serverAddress = InetAddress.getByName("10.0.3.12");
            socket = new Socket(serverAddress, 50000 );
            isConnected = true;
            System.out.println("^^^connected successfully!");

        } catch (UnknownHostException e) {
            System.out.println("^^^Ousted 0");
            e.printStackTrace();
        } catch (IOException e) {
            System.out.println("^^^Ousted 1");
            e.printStackTrace();
        }catch (Exception e)
        {
            System.out.println("^^^Ousted 2");
            e.printStackTrace();
        }

    }


    private void runOnUiThread(Runnable runnable) {
        handler.post(runnable);
    }


    public void closeSocket() {
        try {
            socket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public void setCount(int count) {
        this.count = count;
    }


    public int getCount() {
        return count;
    }


    public void setImageView(ImageView imageView) {
        this.imageView = imageView;
    }


    public ImageView getImageView() {
        return imageView;
    }


    public void setBitmap(Bitmap bitmap) {
        this.bitmap = bitmap;
    }


    public Bitmap getBitmap() {
        return bitmap;
    }


    public void setServerAddress(InetAddress serverAddress) {
        this.serverAddress = serverAddress;
    }


    public InetAddress getServerAddress() {
        return serverAddress;
    }


    public void setSocket(Socket socket) {
        this.socket = socket;
    }


    public Socket getSocket() {
        return socket;
    }


    public int getMATRIX_SIZE() {
        return MATRIX_SIZE;
    }


    public void setIsConnected(boolean isConnected) {
        this.isConnected = isConnected;
    }


    public boolean getIsConnected() {
        return isConnected;
    }


    public void setIsIdling(boolean isIdling) {
        this.isIdling = isIdling;
    }


    public boolean getIsIdling() {
        return isIdling;
    }

    @Override
    protected void finalize() throws Throwable {
        if(socket != null) {
            closeSocket();
        }
        super.finalize();
    }


} // End of class OtherThread