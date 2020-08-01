using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

using NativeWebSocket;

public class Connection : MonoBehaviour
{
    const int NUMBER_GESTURES = 6;
    const int TRAINING = 255;
    const int TRAINED = 1;

    public Button[] gestureTrainButtons;
    public Text[] gestureTrainTexts;

    public Text statusText;
    public Text detectionText;

    WebSocket websocket;

    // Start is called before the first frame update
    async void Start()
    {
        websocket = new WebSocket("ws://beaglebone:1234");

        websocket.OnOpen += () =>
        {
            Debug.Log("Connection open!");
        };

        websocket.OnError += (e) =>
        {
            Debug.Log("Error! " + e);
        };

        websocket.OnClose += (e) =>
        {
            Debug.Log("Connection closed!");
        };

        websocket.OnMessage += (bytes) =>
        {
            // Reading a plain text message
            var message = System.Text.Encoding.UTF8.GetString(bytes);
            Debug.Log("OnMessage! " + message);

            if (message.StartsWith("DATA:"))
            {
                var bytesString = message.Substring("DATA:".Length).Split(' ');
                for (var i = 0; i < NUMBER_GESTURES; i++)
                {
                    gestureTrainTexts[i].text = bytesString[4 + i];
                }

                int trainingStatus = int.Parse(bytesString[2], System.Globalization.NumberStyles.HexNumber);
                if (trainingStatus == TRAINING)
                {
                    statusText.text = "Waiting for training complete...";
                    return;
                }

                if (trainingStatus == TRAINED)
                {
                    statusText.text = "Training Complete, Now Start Detecting Motions...";

                    int detectedGesture = int.Parse(bytesString[3], System.Globalization.NumberStyles.HexNumber);
                    if (detectedGesture >= 0 && detectedGesture < NUMBER_GESTURES)
                    {
                        detectionText.text = "Detected Gesture " + (detectedGesture + 1).ToString();
                        for (var i = 0; i < NUMBER_GESTURES; i++)
                        {
                            gestureTrainTexts[i].color = i == detectedGesture ? Color.red : Color.black;
                        }
                    }
                }
                else
                {
                    SendWebSocketMessage("GATT STOP");
                    statusText.text = "Click a button to train";
                    for (var i = 0; i < NUMBER_GESTURES; i++)
                    {
                        gestureTrainButtons[i].enabled = true;
                    }
                }
            }
        };

        await websocket.Connect();
    }

    void Update()
    {
#if !UNITY_WEBGL || UNITY_EDITOR
        websocket.DispatchMessageQueue();
#endif
    }

    public void TrainGesture(int gestureNumber)
    {
        for (var i = 0; i < NUMBER_GESTURES; i++)
        {
            gestureTrainButtons[i].enabled = false;
        }
        gestureTrainTexts[gestureNumber - 1].text = "...";
        statusText.text = "Wait for LED On to Perform Gesture " + gestureNumber.ToString();

        SendWebSocketMessage(string.Format("GATT WRITE 0012 {0:X2}00", gestureNumber));
    }

    void SendWebSocketMessage(string message)
    {
        if (websocket.State == WebSocketState.Open)
        {
            websocket.SendText(message);
        }
        else
        {
            Debug.Log("Socket not opened");
        }
    }

    private async void OnApplicationQuit()
    {
        await websocket.Close();
    }
}
