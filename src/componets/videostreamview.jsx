import React, { Component } from 'react';
import Config from '../data/config';


class VideoStreamView extends Component {
    state = {
        connected: false,
        ros: null,
        imageString: null,
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
    }


    getStream() {
        var pose = new window.ROSLIB.Topic({
            ros: this.state.ros,
            name: "/video_frames",
            messageType: "std_msgs/String",
        });

        pose.subscribe((msg) => {
            console.log(msg);
            this.setState({
                imageString: this.getData(msg)
            })
        });

    }

    getData(msg) {
        //let TYPED_ARRAY = new Uint8Array(data);
        //let uint8Array = new TextEncoder("utf-8").encode(data);
        //const STRING_CHAR = String.fromCharCode.apply(null, uint8Array.buffer);
        return msg.data;


        // let uint8Array = new TextEncoder("utf-8").encode(data);
        // return URL.createObjectURL(new Blob([uint8Array.buffer], { type: 'image/png' } /* (1) */));
    }

    render() {
        return (<div>
            <h3>Robot Camera</h3>
            <div id='mjpeg'>
                <img id="my_image" src={`data:image/png;base64,${this.state.imageString}`} style={{ height: '100%', width: '100%', objectfit: 'contain' }} alt="loading..." ></img>
            </div>
        </div>);
    }
}

export default VideoStreamView;
