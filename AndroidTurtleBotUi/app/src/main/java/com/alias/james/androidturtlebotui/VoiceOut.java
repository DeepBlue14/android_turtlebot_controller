/*
 * File:   VoiceOut.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class implements text to speech.
 *
 * Last Modified 11/03/2015
 */

package com.alias.james.androidturtlebotui;

import android.annotation.TargetApi;
import android.app.Activity;
import android.os.Build;
import android.speech.tts.TextToSpeech;

/**
 * Created by root on 11/8/15.
 */
public class VoiceOut implements TextToSpeech.OnInitListener
{
    private Activity activity;
    private TextToSpeech tts;

    public VoiceOut(Activity activity)
    {
        this.activity = activity;
        tts = new TextToSpeech(activity, this);
    }


    @Override
    public void onInit(int status)
    {
        if(tts == null)
        {
            tts = new TextToSpeech(activity, this);
        }

        if(status == TextToSpeech.SUCCESS)
        {
            speakOut();
        }
        else
        {
            System.out.println("^^^onInit failed");
        }
    }


    @TargetApi(Build.VERSION_CODES.LOLLIPOP)
    public void speakOut()
    {
        String text = "Testing";
        tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, null);
    }


    public String toString()
    {
        return "***METHOD STUB";
    }


    @Override
    protected void finalize() throws Throwable
    {
        super.finalize();
        tts.shutdown();
    }


} // End of clas VoiceOut