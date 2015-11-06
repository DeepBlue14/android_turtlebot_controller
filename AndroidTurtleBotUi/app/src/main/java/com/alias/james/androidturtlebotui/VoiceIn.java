//If this doesn't work, put code in MainActivity

package com.alias.james.androidturtlebotui;


import java.util.ArrayList;

import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;


/**
 * Created by root on 11/6/15.
 */
public class VoiceIn implements  RecognitionListener
{
    private SpeechRecognizer speech = null;
    private Intent recognizerIntent;
    private String LOG_TAG = "VoiceRecognitionActivity";

    public VoiceIn(Activity activity)
    {
        speech = SpeechRecognizer.createSpeechRecognizer(activity);
        speech.setRecognitionListener(this);
        recognizerIntent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        recognizerIntent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_PREFERENCE,
                "en");
        recognizerIntent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE,
                activity.getPackageName());
        recognizerIntent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
                RecognizerIntent.LANGUAGE_MODEL_WEB_SEARCH);
        recognizerIntent.putExtra(RecognizerIntent.EXTRA_MAX_RESULTS, 3);
    }


    public void start()
    {
        speech.startListening(recognizerIntent);
    }


    public void stop()
    {
        speech.stopListening();
    }


    @Override
    public void onReadyForSpeech(Bundle params)
    {
        Log.i(LOG_TAG, "onReadyForSpeech");
    }


    @Override
    public void onBeginningOfSpeech()
    {
        Log.i(LOG_TAG, "onBeginningOfSpeech");
    }


    @Override
    public void onRmsChanged(float rmsdB)
    {
        Log.i(LOG_TAG, "onRmsChanged: " + rmsdB);
    }


    @Override
    public void onBufferReceived(byte[] buffer)
    {
        Log.i(LOG_TAG, "onBufferReceived: " + buffer);
    }


    @Override
    public void onEndOfSpeech()
    {
        Log.i(LOG_TAG, "onEndOfSpeech");
    }


    @Override
    public void onError(int error)
    {
        String errorMessage = getErrorText(error);
        Log.d(LOG_TAG, "FAILED " + errorMessage);
        System.out.println("^^^FAILED: " + errorMessage);
    }


    @Override
    public void onResults(Bundle results) {
        Log.i(LOG_TAG, "onResults");
        ArrayList<String> matches = results
                .getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
        String text = "";
        // Add all possible matches by confidence level (from most confident to least confident)
        for (String result : matches)
            text += result + "\n";

        System.out.println("^^^I Heard: " + text);
    }


    @Override
    public void onPartialResults(Bundle partialResults)
    {
        Log.i(LOG_TAG, "onPartialResults");
    }


    @Override
    public void onEvent(int eventType, Bundle params)
    {
        Log.i(LOG_TAG, "onEvent");
    }


    public static String getErrorText(int errorCode)
    {
        String message;
        switch (errorCode) {
            case SpeechRecognizer.ERROR_AUDIO:
                message = "Audio recording error";
                break;
            case SpeechRecognizer.ERROR_CLIENT:
                message = "Client side error";
                break;
            case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                message = "Insufficient permissions";
                break;
            case SpeechRecognizer.ERROR_NETWORK:
                message = "Network error";
                break;
            case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
                message = "Network timeout";
                break;
            case SpeechRecognizer.ERROR_NO_MATCH:
                message = "No match";
                break;
            case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
                message = "RecognitionService busy";
                break;
            case SpeechRecognizer.ERROR_SERVER:
                message = "error from server";
                break;
            case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                message = "No speech input";
                break;
            default:
                message = "Didn't understand, please try again.";
                break;
        }
        return message;
    }






} // End of class VoiceIn