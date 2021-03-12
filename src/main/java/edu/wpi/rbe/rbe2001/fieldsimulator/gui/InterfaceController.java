package edu.wpi.rbe.rbe2001.fieldsimulator.gui;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.DecimalFormat;
import java.util.ArrayList;

import edu.wpi.rbe.rbe2001.fieldsimulator.robot.HALDevice;
import javafx.application.Platform;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.ScatterChart;
import javafx.scene.control.Button;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.Label;
import javafx.scene.control.RadioButton;
import javafx.scene.control.SingleSelectionModel;
import javafx.scene.control.Tab;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.chart.XYChart;
import javafx.scene.chart.XYChart.Series;
import javafx.scene.control.CheckBox;

public class InterfaceController {

	@FXML
	private CheckBox use2001;

	@FXML
	private CheckBox useIMU;

	@FXML
	private CheckBox useIR;
	@FXML
	private Label accelx;

	@FXML
	private Label accely;

	@FXML
	private Label accelz;

	@FXML
	private Label gyrox;

	@FXML
	private Label gyroy;

	@FXML
	private Label gyroz;

	@FXML
	private Label gravx;

	@FXML
	private Label gravy;

	@FXML
	private Label gravz;

	@FXML
	private Label eulx;

	@FXML
	private Label euly;

	@FXML
	private Label eulz;
	@FXML
	private Tab connectTab;

	@FXML
	private TextField teamName;

	@FXML
	private Button connectToDevice;

	@FXML
	private Button pidExport;

	@FXML
	private Button velExport;

	@FXML
	private Label robotName;

	@FXML
	private Tab pidVelTab;
	@FXML
	private Tab tab2001Field;
	@FXML
	private TextField kpVel;

	@FXML
	private TextField kdVel;
	
	@FXML
	private TextField velKiField;
	@FXML
	private Button pidConstUpdateVelocity;

	@FXML
	private ChoiceBox<Integer> pidChannelVelocity;

	@FXML
	private TextField setpointVelocity;

	@FXML
	private Button setSetpointVelocity;
	@FXML
	private Button teensyButton;
	@FXML
	private Label velocityVal;

	@FXML
	private Tab pidTab;
	@FXML
	private Tab imutab;
	@FXML
	private Tab irtab;

	@FXML
	private TextField kp;

	@FXML
	private TextField ki;

	@FXML
	private TextField kd;

	@FXML
	private Button pidConstUpdate;

	@FXML
	private TextField setpoint;

	@FXML
	private Button setSetpoint;

	@FXML
	private Label position;

	@FXML
	private Label hardwareOut;

	@FXML
	private Label posHwValue;
	@FXML
	private ChoiceBox<Integer> pidChannel;

	@FXML
	private Button stop;

	@FXML
	private TextArea response;

	@FXML
	private Button send;

	@FXML
	private RadioButton heartBeat;

	@FXML
	private ChoiceBox<String> choiceBoxWeight;

	@FXML
	private ChoiceBox<String> choiceBoxSide;

	@FXML
	private ChoiceBox<String> choiceBoxPos;

	@FXML
	private Button approveButton;

	@FXML // fx:id="setDuration"
	private TextField setDuration; // Value injected by FXMLLoader

	@FXML // fx:id="setType"
	private ChoiceBox<String> setType; // Value injected by FXMLLoader
	@FXML
	private ScatterChart<Double, Double> irChart;
	@FXML
	private LineChart<Double, Double> pidGraph;
	private GraphManager pidManager = null;
	private GraphManager velManager = null;
	@FXML
	private LineChart<Double, Double> pidGraphVel;
//	private ArrayList<XYChart.Series> pidGraphSeriesVel = new ArrayList<>();
//	private ArrayList<XYChart.Series> pidGraphSeries = new ArrayList<>();
	private ObservableList<String> weights = FXCollections.observableArrayList("Aluminum", "Plastic");
	private ObservableList<String> sides = FXCollections.observableArrayList("25", "45");
	private ObservableList<String> pos = FXCollections.observableArrayList("1", "2");
	private double datas[] = null;
	private double irdata[] = null;

	private DecimalFormat formatter = new DecimalFormat();

	static InterfaceController me;
	private static HALDevice robot;

	private int numPIDControllers = -1;
	private int currentIndex = 0;
	private static final int numPIDControllersOnDevice = 3;
	private File lastSearchedName = new File(
			System.getProperty("user.home") + "/" + "rbeFieldControllerLastSearchedRobot.txt");
	private CSVManager csv=new CSVManager();
	private double Azimuth =0;
	@FXML
	private void initialize() {
		me = this;
		formatter.setMaximumFractionDigits(6);

		assert connectTab != null : "fx:id=\"connectTab\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert teamName != null : "fx:id=\"teamName\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert connectToDevice != null : "fx:id=\"connectToDevice\" was not injected: check your FXML file 'MainScreen.fxml'.";

		assert pidTab != null : "fx:id=\"pidTab\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert pidGraph != null : "fx:id=\"pidGraph\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert kp != null : "fx:id=\"kp\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert ki != null : "fx:id=\"ki\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert kd != null : "fx:id=\"kd\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert pidConstUpdate != null : "fx:id=\"pidConstUpdate\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert pidChannel != null : "fx:id=\"pidChannel\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert setpoint != null : "fx:id=\"setpoint\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert setSetpoint != null : "fx:id=\"setSetpoint\" was not injected: check your FXML file 'MainScreen.fxml'.";
		assert position != null : "fx:id=\"position\" was not injected: check your FXML file 'MainScreen.fxml'.";

		pidManager = new GraphManager(pidGraph, 3);
		velManager = new GraphManager(pidGraphVel, 3);

		irChart.getData().add(new XYChart.Series());
		irChart.getData().add(new XYChart.Series());
		irChart.getData().add(new XYChart.Series());
		irChart.getData().add(new XYChart.Series());

		choiceBoxWeight.setValue(weights.get(0));
		choiceBoxWeight.setItems(weights);
		choiceBoxSide.setValue("25");
		choiceBoxSide.setItems(sides);
		choiceBoxPos.setValue("1");
		choiceBoxPos.setItems(pos);

		choiceBoxWeight.getSelectionModel().select(weights.get(0));

		stop.setDisable(true);
		// PLE.setDisable(true);
		// RHE.setDisable(true);
		send.setDisable(true);
		approveButton.setDisable(true);
		if (lastSearchedName.exists()) {
			try {
				String text = new String(Files.readAllBytes(lastSearchedName.toPath()));
				Platform.runLater(() -> teamName.setText(text));
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	private void connectToDevice() {
		if (getRobot() == null) {
			connectToDevice.setDisable(true);
			new Thread(() -> {
				String name = teamName.getText();
				try {
					if (!lastSearchedName.exists())
						lastSearchedName.createNewFile();
					BufferedWriter writer = new BufferedWriter(new FileWriter(lastSearchedName));
					writer.write(name);
					writer.close();
					System.out.println("Writing to " + lastSearchedName.getAbsolutePath());
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

				try {
					robot=new HALDevice(name);
					robot.connect();
					getRobot().send();
					// getFieldSim().setReadTimeout(1000);
//					if (getRobot() != null) {
//						Platform.runLater(() -> connectToDevice.setDisable(true));
//						Platform.runLater(() -> teensyButton.setDisable(true));
//						Platform.runLater(() -> {
//							robotName.setText("Connected to: "+name);
//							pidTab.setDisable(false);
//							pidVelTab.setDisable(false);
//						});
//
//					}
				} catch (Exception ex) {
					ex.printStackTrace();
					Platform.runLater(() -> robotName.setText( "Robot Not Found!"));
				}
				if (getRobot() == null) {
					Platform.runLater(() -> connectToDevice.setDisable(false));
				}

			}).start();
		}
	}

	@FXML
	void onConnect() {
		System.out.println("onConnect");
		connectToDevice();
	}

	@FXML
	void connectTeensy() {
		
	}

	@FXML
	void onSetGains() {
		double kpv = Double.parseDouble(kp.getText());
		double kiv = Double.parseDouble(ki.getText());
		double kdv = Double.parseDouble(kd.getText());
		// for (int i = 0; i < numPIDControllers; i++)
		//robot.setPidGains(currentIndex, kpv, kiv, kdv);
	}

	@FXML
	void onSetSetpoint() {
		clearGraph();
//		robot.setPidSetpoint(Integer.parseInt(setDuration.getText()),
//				setType.getSelectionModel().getSelectedItem().equals("LIN") ? 0 : 1, currentIndex,
//				Double.parseDouble(setpoint.getText()));

	}

	public HALDevice getRobot() {
		return robot;
	}

	@SuppressWarnings("unchecked")
	private void updateIR(double[] pos) {
		for (int i = 0; i < 4; i++) {
			double x = pos[i * 2];
			double y = pos[i * 2 + 1];
			Series e = irChart.getData().get(i);
			e.getData().clear();
			e.getData().add(new XYChart.Data(x, y));
		}
	}


	private void setUpPid() {

	}

	private void clearGraph() {
		if (pidTab.isSelected()) {
			currentIndex = pidChannel.getSelectionModel().getSelectedItem().intValue();
		}
		if (pidVelTab.isSelected()) {
			SingleSelectionModel<Integer> model = pidChannelVelocity.getSelectionModel();
			Integer item = model.getSelectedItem();
			// try {
			currentIndex = item.intValue();

		}
		System.out.println("Set to channel " + currentIndex);
		pidManager.clearGraph(currentIndex);
		velManager.clearGraph(currentIndex);
		//robot.updatConfig();
	}

	public static void disconnect() {
		if (me.getRobot() != null)
			me.getRobot().disconnect();
	}

	@FXML
	void onSetVelocity() {
		clearGraph();
		double vel = Double.parseDouble(setpointVelocity.getText());
//		if (vel != 0)
//			robot.setVelocity(currentIndex, Double.parseDouble(setpointVelocity.getText()));
//		else
//			robot.stop(currentIndex);

	}

	@FXML
	void onSetGainsVelocity() {
		double kpv = Double.parseDouble(kpVel.getText());
		double kiv = Double.parseDouble(velKiField.getText());
		double kdv = Double.parseDouble(kdVel.getText());
		//robot.setVelocityGains(currentIndex, kpv, kiv,kdv);
	}

	@FXML
	void onApprove() {
//		System.out.println("approve");
//		if (rbe2001 != null)
//			rbe2001.approve();

	}

	@FXML
	void sendLocation() {
		System.out.println("sendLocation");
		double material;
		if (choiceBoxWeight.getSelectionModel().getSelectedItem().contains(weights.get(0))) {
			material = 1;
		} else {
			material = 2;
		}
		double angle = Double.parseDouble(choiceBoxSide.getSelectionModel().getSelectedItem());
		double position = Double.parseDouble(choiceBoxPos.getSelectionModel().getSelectedItem());
//		if (rbe2001 != null)
//			rbe2001.pickOrder(material, angle, position);

	}

	@FXML
	void start() {
		System.out.println("start");
//		if (rbe2001 != null)
//			rbe2001.clearFaults();

	}

	@FXML
	void stop() {
		System.out.println("stop");
//		if (rbe2001 != null)
//			rbe2001.estop();

	}

	@FXML
	void onPidExport() {
		try {
			csv.writeToFile();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@FXML
	void onVelExport() {
		try {
			csv.writeToFile();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
