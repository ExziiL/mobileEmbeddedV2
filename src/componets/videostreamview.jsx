import React, { Component } from 'react';
import * as mobilenet from "@tensorflow-models/mobilenet";
import * as tf from "@tensorflow/tfjs";
import Config from '../data/config';
import * as jpeg from 'jpeg-js'


class VideoStreamView extends Component {
    state = {
        connected: false,
        ros: null,
        imageString: null,
        model: null,
        detected: false,
    }

    constructor() {
        super();
        this.getStream = this.getStream.bind(this);
        this.state.ros = new window.ROSLIB.Ros();
        console.log(this.state.ros);
    }

    reconnect() {
        if (this.state.connected === false) {
            console.log(this.state.connected);
            console.log('Reconnecting');
            try {
                try {
                    try {
                        this.state.ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`).onerror(function (e) {
                            console.log("Error caught by connect: ");
                            console.log(e);
                        });
                    } catch (error) {
                        console.log("Connection problem");
                    }
                } catch (error) {
                    console.log("Timesout Error");
                }
            } catch (error) {
                console.log("Error");
            }

        }
    }

    init_connection() {
        window.onerror = function (e) {
            console.log("error handled", e.type);
            console.log("error handled", e);
        };

        setInterval(() => {
            this.reconnect();
        }, 5000);

        this.state.ros.on("connection", () => {
            console.log("[Statitics]]Connection established successfully");
            this.setState({ connected: true });
            this.getStream();

        });
        this.state.ros.on("close", (error) => {
            console.log(error);
            console.log("Connection closed");
            this.setState({ connected: false });
        });

        try {
            this.state.ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`).onerror(function (e) {
                console.log("Error caught by connect: ");
                console.log(e);
            });
        } catch (error) {
            console.log("Connection problem");
        }

    }
    componentDidMount() {
        this.init_connection();
        tf.ready().then(() => {
            this.loadModel();
        });
        // Call predictionFunction every second
        setInterval(() => {
            this.predictionFunction();
        }, 2000);
    }

    async loadModel() {
        try {
            const mobilnet = await mobilenet.load();
            this.setState({ model: mobilnet })
            console.log("setloadedModel");
        } catch (err) {
            console.log(err);
            console.log("failed load model");
        }
    }


    getStream() {
        var pose = new window.ROSLIB.Topic({
            ros: this.state.ros,
            name: "/video_frames",
            messageType: "std_msgs/String",
        });

        pose.subscribe((msg) => {
            //console.log(msg);
            this.setState({
                imageString: this.getData(msg)
            })
        });

    }

    getData(msg) {

        return msg.data;
    }

    async predictionFunction() {
        if (this.state.imageString != null && this.state.model != null) {
            const imageTensor = this.imageToTensor(this.base64ToArrayBuffer(this.state.imageString));
            const predictions = await this.state.model.classify(imageTensor);
            let found = false;
            if (predictions.length > 0) {
                // setPredictionData(predictions);
                //console.log(predictions);
                for (let n = 0; n < predictions.length; n++) {
                    // Check scores
                    //console.log(n);
                    if (predictions[n].probability > 0.2) {
                        if (predictions[n].className.match(/mug|cup/))
                        {   
                            console.log("detected");
                            found = true;
                          //  this.setState({ detected: true });
                            document.querySelector('#mjpeg').style.border = "10px solid green";
                        }
                    }
                }
            }
            if(!found){
                document.querySelector('#mjpeg').style.border = "10px solid red";
              //  this.setState({ detected: false });
            }
        }
    }



    base64ToArrayBuffer(base64) {
        var binaryString = atob(base64);
        var bytes = new Uint8Array(binaryString.length);
        for (var i = 0; i < binaryString.length; i++) {
            bytes[i] = binaryString.charCodeAt(i);
        }
        return bytes.buffer;
    }

    imageToTensor(rawImageData) {
        const TO_UINT8ARRAY = true
        const { width, height, data } = jpeg.decode(rawImageData, TO_UINT8ARRAY)
        // Drop the alpha channel info for mobilenet
        const buffer = new Uint8Array(width * height * 3)
        let offset = 0 // offset into original data
        for (let i = 0; i < buffer.length; i += 3) {
            buffer[i] = data[offset]
            buffer[i + 1] = data[offset + 1]
            buffer[i + 2] = data[offset + 2]

            offset += 4
        }

        return tf.tensor3d(buffer, [height, width, 3])
    }

    render() {
        return (<div>
            <h3>Robot Camera</h3>
            {this.state.detected && <h4>Detected</h4>}
            <div id='mjpeg'>
                <img id="my_image" src={`data:image/png;base64,${this.state.imageString}`} style={{ height: '100%', width: '100%', objectfit: 'contain' }} alt="loading..." ></img>
            </div>
        </div>);
    }
}

export default VideoStreamView;
