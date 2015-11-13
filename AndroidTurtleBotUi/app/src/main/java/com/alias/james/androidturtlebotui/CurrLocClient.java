/*
 * File:   CamClient.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class contains a thread which will receive updates as to the robots
 *                   current location.
 *
 *
 * Last Modified 11/08/2015
 */


package com.alias.james.androidturtlebotui;


import android.graphics.Bitmap;
import android.graphics.PointF;
import android.os.Handler;
import android.widget.ImageView;

import java.io.IOException;
import java.io.InputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;


/**
 * Created by root on 11/12/15.
 */
public class CurrLocClient implements Runnable {

    //TODO: main() should notify this thread which fragment is loaded
    enum CurrentFrag {
        MAP,
        CAM,
        TELEOP
    }

    private static CurrentFrag currentFrag;
    private char DELIMITER = '|';
    private PointF robotCurrPos = new PointF();
    private ImageView imageView;
    private Bitmap bitmap;
    private InetAddress serverAddress = null;
    private Socket socket;
    private final int MESSAGE_SIZE = 8;
    private boolean isConnected = false;
    private boolean isIdling = false;
    Handler handler;

    public CurrLocClient() {

    }


    @Override
    public void run() {

        byte[] buffer = new byte[MESSAGE_SIZE];
        int currPos = 0;
        int bytesRead = 0;
        InputStream sub;
        String posAsStr;

        while (true) {
            if (!isIdling) {

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

                        //convert buffer to string
                        posAsStr = new String(buffer, "UTF-8");

                        //TODO: convert string to floats
                        String tmpStr1 = new String();
                        String tmpStr2 = new String();
                        boolean isFirstFloat = true;
                        for(int i = 0; i < posAsStr.length(); i++) {
                            if(posAsStr.charAt(i) != DELIMITER && isFirstFloat) {
                                tmpStr1 += posAsStr.charAt(i);
                            } else {
                                isFirstFloat = false;
                                tmpStr2 += posAsStr.charAt(i);
                            }
                        }
                        robotCurrPos.x = Float.parseFloat(tmpStr1);
                        robotCurrPos.y = Float.parseFloat(tmpStr2);
                        System.out.println("^^^recieved location from robot: (" + robotCurrPos.x + ", " + robotCurrPos.y + ")");

                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                //imageView.setImageBitmap(bitmap);
                            }
                        });
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
            serverAddress = InetAddress.getByName("10.0.4.6");
            socket = new Socket(serverAddress, 50001);
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


    public int getMESSAGE_SIZE() {
        return MESSAGE_SIZE;
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


}