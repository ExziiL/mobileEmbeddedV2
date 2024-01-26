import React, { Component } from "react";
import Connection from "../componets/connection";
import NavigationOptions from "../componets/mapgoal";
import RobotStatistics from "../componets/robotstatistics";
import Teleoperation from "../componets/teleoperation";
import VideoStreamView from "../componets/videostreamview";

class Home extends Component {
	state = {};
	render() {
		return (
			<div className="">
				<Connection />
				<div className="col mt-3">
					<div className="d-md-flex align-items-stretch">
						<VideoStreamView className="" />
						<RobotStatistics className="" />
					</div>
					<div className="d-md-flex">
						<NavigationOptions />
					</div>
				</div>
				<Teleoperation />
			</div>
		);
	}
}

export default Home;
