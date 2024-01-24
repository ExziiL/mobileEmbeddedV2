import ls from "local-storage";
// eslint-disable-next-line
import { MapPin, Play, Signpost } from "lucide-react";
import React, { Component } from "react";
import { Button } from "react-bootstrap";
import { v4 as uuid4 } from "uuid";
import Config from "../data/config";
import paths from "../data/paths";

class MapGoal extends Component {
	state = {
		connected: false,
		ros: null,
		isMap: false,
		goal_list: [],
		goalid_list: [],
	};

	componentDidMount() {
		this.init_connection();
		let goals = ls.get("goal_list");
		let newGoals = [];
		try {
			for (var i = 0; i < goals.length; i++) {
				try {
					var test = goals[i].id;
					if (typeof test != "undefined") {
						newGoals.push(goals[i]);
					}
				} catch (error) {}
			}
		} catch (error) {}
		if (newGoals.length === 0) {
			ls.set("goal_list", []);
			ls.set("goalid_list", []);
		}
		this.setState({
			goal_list: newGoals,
			goalid_list: ls.get("goalid_list") || [],
		});
	}

	updateStorage() {
		ls.set("goal_list", this.state.goal_list);
		ls.set("goalid_list", this.state.goalid_list);
	}

	constructor() {
		super();
		this.updateStorage = this.updateStorage.bind(this);
		this.setGoal = this.setGoal.bind(this);
		this.cancelGoal = this.cancelGoal.bind(this);
		this.deleteGoal = this.deleteGoal.bind(this);
		this.newLocation = this.newLocation.bind(this);
		this.addNewLocation = this.addNewLocation.bind(this);
		this.state.ros = new window.ROSLIB.Ros();
		this.driveRecordedPath = this.driveRecordedPath.bind(this);
		console.log(this.state.ros);
	}

	reconnect() {
		if (this.state.connected === false) {
			console.log(this.state.connected);
			console.log("Reconnecting");
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
			console.log("[Map Goal]Connection established successfully");
			this.setState({ connected: true });
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

	setGoal(id) {
		var goal_data = null;
		var goal_list = this.state.goal_list;
		for (var i = 0; i < goal_list.length; i++) {
			if (goal_list[i].id === id) {
				goal_data = goal_list[i].pose;
			}
		}
		if (goal_data === null) {
			return 0;
		}
		console.log("Navigating");
		var goal = new window.ROSLIB.Topic({
			ros: this.state.ros,
			name: Config.GOAL_TOPIC,
			messageType: "geometry_msgs/PoseStamped",
		});
		var goal_msg = new window.ROSLIB.Message(goal_data);
		goal.publish(goal_msg);
	}

	deleteGoal(id) {
		try {
			for (var i = 0; i < this.state.goal_list.length; i++) {
				try {
					if (this.state.goal_list[i].id === id) {
						console.log(i);
						delete this.state.goal_list[i];
						break;
					}
				} catch (error) {}
			}
			this.setState({ goal_list: this.state.goal_list });
			this.updateStorage();
		} catch (error) {}
	}

	newLocation() {
		var name = window.prompt("Enter location name");
		try {
			if (name.length > 0) {
				this.addNewLocation(name);
			}
		} catch (error) {
			return "";
		}
		return "";
	}

	cancelGoal() {
		var odom = new window.ROSLIB.Topic({
			ros: this.state.ros,
			name: "/odom",
			messageType: "nav_msgs/Odometry",
		});
		var added = false;
		var newgoal = null;
		odom.subscribe((msg) => {
			if (added === false) {
				added = true;
				newgoal = {
					pose: {
						header: {
							frame_id: "map",
						},
						pose: msg.pose.pose,
					},
				};
				var goal = new window.ROSLIB.Topic({
					ros: this.state.ros,
					name: Config.GOAL_TOPIC,
					messageType: "geometry_msgs/PoseStamped",
				});
				var goal_msg = new window.ROSLIB.Message(newgoal.pose);
				goal.publish(goal_msg);
			}
			odom.unsubscribe();
		});

		this.setState();
	}

	addNewLocation(goalname) {
		var id = uuid4();

		var odom = new window.ROSLIB.Topic({
			ros: this.state.ros,
			name: "/odom",
			messageType: "nav_msgs/Odometry",
		});
		var added = false;
		var newgoal = null;
		odom.subscribe((msg) => {
			if (added === false) {
				added = true;
				newgoal = {
					id: id,
					name: goalname,
					pose: {
						header: {
							frame_id: "map",
						},
						pose: msg.pose.pose,
					},
				};
				if (this.state.goalid_list.includes(id) === false) {
					console.log(this.state.goalid_list.includes(id));
					this.state.goal_list.push(newgoal);
					this.state.goalid_list.push(id);
					this.setState({ goal_list: this.state.goal_list });
					this.updateStorage();
				}
			}
			console.log(this.state.goal_list);
			console.log(msg);
			odom.unsubscribe();
		});

		this.setState();
	}

	async driveRecordedPath(id) {
		const sleep = (delay) => new Promise((resolve) => setTimeout(resolve, delay));
		let path = paths["path" + id];
		let twists = [];
		let reverseTwists = [];

		path.forEach((coordinate) => {
			let message = {
				linear: coordinate.linear,
				angular: coordinate.angular,
			};
			twists.push(new window.ROSLIB.Message(message));

			let reverseMessage = JSON.parse(JSON.stringify(message));
			reverseMessage.linear.x = -message.linear.x;
			reverseMessage.angular.z = -message.angular.z;
			reverseTwists.push(new window.ROSLIB.Message(reverseMessage));
		});

		twists = twists.concat(reverseTwists);
		// for (let i=reverseTwists.length; i>=0; i--) {
		// 	twists.push(reverseTwists.pop());
		// }

		for (const twist of twists) {
			let cmd_vel = new window.ROSLIB.Topic({
				ros: this.state.ros,
				name: Config.CMD_VEL_TOPIC,
				messageType: "geometry_msgs/Twist",
			});
			cmd_vel.publish(twist);
			await sleep(200); // delay between 400 and 500 ms
		}
	}

	render() {
		return (
			this.state.connected && (
				<div
					className="border w-100 mx-md-2 mb-2 p-2"
					style={{ minHeight: "150px" }}
				>
					<h3 className="d-flex gap-3 align-items-center">
						<Signpost
							size={28}
							strokeWidth={2}
						/>
						Navigation Options
					</h3>
					<div className="d-flex flex-column w-auto pt-2 gap-3">
						<Button
							className=" w-50"
							onClick={() => this.driveRecordedPath(1)}
							variant="outline-success"
						>
							<div className="d-flex align-items-center gap-2">
								<Play
									size={20}
									strokeWidth={1.5}
								/>
								1. Table
							</div>
						</Button>
						<Button
							className=" w-50"
							onClick={() => this.driveRecordedPath(2)}
							variant="outline-success"
						>
							<div className="d-flex align-items-center gap-2">
								<Play
									size={20}
									strokeWidth={1.5}
								/>
								2. Table
							</div>
						</Button>
						<Button
							className=" w-50"
							onClick={this.newLocation}
							variant="outline-success"
						>
							<div className="d-flex align-items-center gap-2">
								<Play
									size={20}
									strokeWidth={1.5}
								/>
								3. Table
							</div>
						</Button>
						<Button
							className="w-50"
							onClick={this.newLocation}
							variant="outline-success"
						>
							<div className="d-flex align-items-center gap-2">
								<Play
									size={20}
									strokeWidth={1.5}
								/>{" "}
								4. Table
							</div>
						</Button>
					</div>
					đ
				</div>
			)
		);
	}
}

export default MapGoal;
