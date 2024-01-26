import { Mic, MicOff } from "lucide-react";
import React from "react";
import { Button } from "react-bootstrap";
import SpeechRecognition, { useSpeechRecognition } from "react-speech-recognition";
import { Bars } from "svg-loaders-react";
import wtn from "words-to-numbers";

const OPENAI_API_KEY = process.env.REACT_APP_OPEN_AI_KEY;

const Speech = () => {
	const commands = [
		{
			command: "go to table *",
			callback: async (number: string) => {
				handleGoToTable(number);
			},
		},
		{
			command: "table *",
			callback: async (number: string) => {
				console.log("Table number: ", number);
				handleGoToTable(number);
			},
		},
		{
			command: "Tisch *",
			callback: async (number: string) => {
				handleGoToTable(number);
			},
		},
	];

	const handleGoToTable = (spokenNumber: string) => {
		const number = typeof spokenNumber == "string" ? wtn(spokenNumber) : spokenNumber;
		console.log("Number:", number);
		console.log("Spoken number:", spokenNumber);
		if (number === undefined) {
			alert("Sorry, I didn't understand that. Please try again.");
			return;
		} else {
			alert(number);
		}
	};

	const { transcript, listening, resetTranscript, browserSupportsSpeechRecognition, isFuzzyMatch } = useSpeechRecognition({ commands });
	const [thingsPreviousSaid, setThingsPreviousSaid] = React.useState<string[]>([]);
	const [isReadyForResponse, setIsReadyForResponse] = React.useState(false);
	const [openAiResponses, setOpenAiResponses] = React.useState<string[]>([]);
	const [isCurrentlyResponding, setIsCurrentlyResponding] = React.useState(false);

	React.useEffect(() => {
		if (isReadyForResponse && transcript) {
			handleNewTranscript();
		}
		// eslint-disable-next-line react-hooks/exhaustive-deps
	}, [transcript, isReadyForResponse]);

	const speak = (text: string) => callOpenAiTTS(text);

	const callOpenAiTTS = async (text: string) => {
		try {
			const response = await fetch("https://api.openai.com/v1/audio/speech", {
				method: "POST",
				headers: {
					Authorization: `Bearer ${OPENAI_API_KEY}`,
					"Content-Type": "application/json",
				},
				body: JSON.stringify({
					model: "tts-1",
					voice: "alloy",
					input: text,
				}),
			});

			if (!response.ok) {
				throw new Error(`HTTP error! status: ${response.status}`);
			}

			const audioBlob = await response.blob();
			const audioUrl = URL.createObjectURL(audioBlob);

			const audio = new Audio(audioUrl);
			setIsCurrentlyResponding(true);
			audio.play();
			audio.onended = () => {
				setIsCurrentlyResponding(false);
				URL.revokeObjectURL(audioUrl);
			};
			return audioBlob;
		} catch (error) {
			console.error("Error calling OpenAI TTS:", error);
		}
	};

	const handleNewTranscript = async () => {
		setTimeout(async () => {
			const latestTranscript = transcript;
			resetTranscript();
			setThingsPreviousSaid([...thingsPreviousSaid, latestTranscript]);

			const response = await askOpenAi(latestTranscript);
			const answer = response?.choices[0].message.content;

			console.log("Previous things said:", thingsPreviousSaid);
			console.log("OpenAI response:", answer);

			setOpenAiResponses((prev) => [...prev, answer]);

			await speak(answer);
			setIsReadyForResponse(false);
		}, 3000);
	};

	const startListeningWithTimeout = () => {
		SpeechRecognition.startListening({ language: "en-US", continuous: true, interimResults: true });
		setIsReadyForResponse(false);

		setTimeout(() => {
			SpeechRecognition.stopListening();
			setIsReadyForResponse(true);
			setIsCurrentlyResponding(true);
		}, 5000);
	};

	if (!browserSupportsSpeechRecognition) {
		return <span>Browser doesn't support speech recognition.</span>;
	}

	const askOpenAi = async (currentTranscript: any) => {
		try {
			if (isCurrentlyResponding) {
				return;
			}

			const interactionHistory = thingsPreviousSaid.reduce((acc: any, thing, index) => {
				acc.push({ role: "user", content: thing });
				if (openAiResponses[index]) {
					acc.push({ role: "assistant", content: openAiResponses[index] });
				}
				return acc;
			}, []);

			const response = await fetch("https://api.openai.com/v1/chat/completions", {
				method: "POST",
				headers: {
					"Content-Type": "application/json",
					Authorization: "Bearer " + OPENAI_API_KEY,
				},
				body: JSON.stringify({
					model: "gpt-4-1106-preview",
					messages: [
						{
							role: "system",
							// TODO: Den content hier so einstellen, dass er von kunden und mitarbeitern input bekommen kann und dementsprechend darauf reagiert
							content:
								"Your name is Robi, you are a personal assistant robot and you speak to a human. Your housing and hardware are based on the Turtlebot3. Your field of application is catering. You will be used to take plates and drink containers to the customer's table and collect the used items again.  A trailer is attached to your housing, on which customers can place and remove their plates and cups. You must be careful not to drive too fast. You are also equipped with a camera for object recognition. This camera allows you to recognise whether a plate is full or empty. If there is a filled cup on your trailer, you must drive more slowly and start and stop more carefully so that nothing leaks out. Your function as a robot is also to talk to customers and answer their questions. If you don't know a question or the customer, i.e. the user, asks you something that you can't answer, you can come up with any answer you like. However, this answer should match the customer's question and address it carefully. Always be polite to the customer and give short but precise answers. You are working for the restaurant 'Robi's Restaurant'. You are a waiter and you are responsible for the food and drinks. You will be asked questions from the customer and you will answer them. You will also be asked to do something from the workers in the restaurant, for example to bring food and drinks to the customer's table. When a worker asks you to do something, you will do it and tell them before, what you will do. If a worker tells you to bring something to a specific customer table, you will tell the worker to which customer table you will bring the food or drinks. If a user inputs a number, please output the number as a Number and not as a string",
						},
						...interactionHistory,
						{ role: "user", content: currentTranscript },
					],
				}),
			});

			const data = await response.json();
			return data;
		} catch (error) {
			console.log(error);
		}
	};

	return (
		<div className="text-center mx-2">
			<div className="d-flex gap-2 justify-content-center">
				Microphone:
				{listening ? (
					<div className="d-flex gap-1 align-items-center">
						<Mic size={18} />
						On
					</div>
				) : (
					<div className="d-flex gap-1 align-items-center">
						<MicOff size={18} />
						Off
					</div>
				)}
			</div>

			<div className="d-flex justify-content-center pt-3 gap-4">
				<Button
					disabled={isCurrentlyResponding || listening}
					onClick={startListeningWithTimeout}
					variant="primary"
				>
					<div className="d-flex gap-2">
						{listening && (
							<Bars
								stroke="#fff"
								fill="#fff"
								style={{ width: 24, height: 24 }}
							/>
						)}
						Start
					</div>
				</Button>
				<Button
					disabled={isCurrentlyResponding}
					onClick={() => SpeechRecognition.stopListening()}
					variant="secondary"
				>
					Stop
				</Button>
			</div>

			{listening && <p className="mt-3">Robi is listening...</p>}

			{transcript && (
				<div
					className="border mt-3 mx-auto border-secondary mx-2 py-1"
					style={{ maxWidth: "500px" }}
				>
					Your input:
					<p>{transcript}</p>
				</div>
			)}

			{isCurrentlyResponding && (
				<div className="py-4">
					Robi is responding...{" "}
					<div
						className=""
						style={{ fontSize: "13px", opacity: "0.8" }}
					>
						Please be patient
					</div>
				</div>
			)}
		</div>
	);
};
export default Speech;
