import React, { Component } from "react";
import { Col, Row } from "react-bootstrap";
import Connection from "../componets/connection";
import MapGoal from "../componets/mapgoal";
import MapView from "../componets/mapview";
import RobotStatistics from "../componets/robotstatistics";
import Teleoperation from "../componets/teleoperation";
import VideoStreamView from "../componets/videostreamview";

class Home extends Component {
	state = {};
	render() {
		return (
			<div className="d-flex flex-column">
				<Row className="nomargin  nopadding">
					<Col className="nomargin nopadding box-border">
						<Connection />
					</Col>
				</Row>
				<Row className="nomargin nopadding">
					<Col className="nomargin nopadding box-border">
						<VideoStreamView className="box-border" />
						<RobotStatistics className="nomargin box-border" />
						<MapGoal />
					</Col>
					<Col className="nomargin nopadding box-border ">
						<MapView />
					</Col>
				</Row>
				<Row className="absolute">
					<Col>
						<Teleoperation />
					</Col>
				</Row>
			</div>
		);
	}
}

export default Home;
