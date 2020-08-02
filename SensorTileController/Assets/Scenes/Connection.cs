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

    public float axisMagnitude = 0.5f;
    Vector3 axisInput;

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
            Debug.LogError("Error! " + e);
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

            if (message.StartsWith("ERROR:"))
            {
                SendWebSocketMessage("GATT STOP");
                statusText.text = message.Substring("ERROR:".Length);
                for (var i = 0; i < NUMBER_GESTURES; i++)
                {
                    gestureTrainButtons[i].enabled = true;
                }
            }
            else if (message.StartsWith("DATA:"))
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
                    for (var i = 0; i < NUMBER_GESTURES; i++)
                    {
                        gestureTrainTexts[i].color = i == detectedGesture ? Color.red : Color.black;
                    }

                    if (detectedGesture >= 0 && detectedGesture < NUMBER_GESTURES)
                    {
                        detectionText.text = "Detected Gesture: " + (detectedGesture + 1).ToString();
                        switch (detectedGesture)
                        {
                            case 0:
                                //axisInput.x = 0;
                                axisInput.y += axisMagnitude;
                                break;
                            case 1:
                                //axisInput.x = 0;
                                axisInput.y -= axisMagnitude;
                                break;
                            case 2:
                                axisInput.x -= axisMagnitude;
                                //axisInput.y = 0;
                                break;
                            case 3:
                                axisInput.x += axisMagnitude;
                                //axisInput.y = 0;
                                break;
                            //default:
                            //    axisInput.x = 0;
                            //    axisInput.y = 0;
                            //    break;
                        }
                        axisInput.x = Mathf.Clamp(axisInput.x, -1, 1);
                        axisInput.y = Mathf.Clamp(axisInput.y, -1, 1);
                    }
                    else
                    {
                        detectionText.text = "Detected Gesture: None";
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
        if (websocket.State == WebSocketState.Open)
            websocket.DispatchMessageQueue();
#endif
    }

    public void TrainGesture(int gestureNumber)
    {
        if (gestureNumber > 0 && gestureNumber <= NUMBER_GESTURES)
        {
            for (var i = 0; i < NUMBER_GESTURES; i++)
            {
                gestureTrainButtons[i].enabled = false;
            }
            gestureTrainTexts[gestureNumber - 1].text = "...";
            statusText.text = "Wait for LED On to Perform Gesture " + gestureNumber.ToString();
        }

        SendWebSocketMessage(string.Format("GATT WRITE 0012 {0:X2}00", gestureNumber));
    }

    public void SendWebSocketMessage(string message)
    {
        if (websocket.State == WebSocketState.Open)
        {
            websocket.SendText(message);
        }
        else
        {
            Debug.LogWarning("Socket not opened");
        }
    }

    private async void OnApplicationQuit()
    {
        await websocket.Close();
    }

    public float GetAxis(string axis)
    {
        if (axis == "Vertical")
        {
            return axisInput.y;
        }
        else if (axis == "Horizontal")
        {
            return axisInput.x;
        }
        return 0;
    }
}
