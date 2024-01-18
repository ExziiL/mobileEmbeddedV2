import Speech from "./audio/speech";
import Footer from "./componets/footer";
import Header from "./componets/header";
import RoutesContainer from "./routes/routes-container";

function App() {
	return (
		<div className="App">
			<Header />
			<RoutesContainer />
			<Footer />

			<Speech />
		</div>
	);
}

export default App;
