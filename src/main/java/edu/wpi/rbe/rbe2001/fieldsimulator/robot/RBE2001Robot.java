package edu.wpi.rbe.rbe2001.fieldsimulator.robot;

import java.util.Arrays;
import edu.wpi.SimplePacketComs.BytePacketType;
import edu.wpi.SimplePacketComs.FloatPacketType;
import edu.wpi.SimplePacketComs.PacketType;
import edu.wpi.SimplePacketComs.device.UdpDevice;

public class RBE2001Robot extends UdpDevice  implements SimplePIDRobot,IRBE2001Robot,IRBE2002Robot{
	private FloatPacketType IMU = new FloatPacketType(1804, 64);

	private FloatPacketType getIR = new FloatPacketType(1590, 64);

	private FloatPacketType setSetpoint = new FloatPacketType(1848, 64);
	private FloatPacketType pidStatus = new FloatPacketType(1910, 64);
	private FloatPacketType getConfig = new FloatPacketType(1857, 64);
	private FloatPacketType setConfig = new FloatPacketType(1900, 64);
	private PacketType estop = new BytePacketType(1989, 64);
	private PacketType getStatus = new BytePacketType(2012, 64);
	private PacketType clearFaults = new BytePacketType(1871, 64);
	private PacketType pickOrder = new FloatPacketType(1936, 64);
	private PacketType approve = new BytePacketType(1994, 64);
	private PacketType SetPIDVelocity = new FloatPacketType(1811, 64);
	private PacketType SetPDVelocityConstants = new FloatPacketType(1810, 64);
	private PacketType GetPIDVelocity = new FloatPacketType(1822, 64);
	private PacketType GetPDVelocityConstants = new FloatPacketType(1825, 64);

	private byte[] status = new byte[1];
	private double[] pickOrderData = new double[3];
	private double[] driveStatus = new double[1];
	double[] numPID = new double[1];
	double[] pidConfigData = new double[15];
	double[] pidVelConfigData = new double[15];

	private double[] piddata;
	private double[] veldata;
	private int myNum = -1;

	public RBE2001Robot(String add, int numPID) throws Exception {
		super(add);
		myNum = numPID;
		SetPIDVelocity.waitToSendMode();
		SetPDVelocityConstants.waitToSendMode();
		GetPIDVelocity.pollingMode();
		GetPDVelocityConstants.oneShotMode();

		getConfig.oneShotMode();
		setConfig.waitToSendMode();
		setSetpoint.waitToSendMode();

		for (PacketType pt : Arrays.asList(pidStatus, getConfig, setConfig, setSetpoint, SetPIDVelocity,
				SetPDVelocityConstants, GetPIDVelocity, GetPDVelocityConstants)) {
			addPollingPacket(pt);
		}

		addEvent(GetPDVelocityConstants.idOfCommand, () -> {
			try {
				readFloats(GetPDVelocityConstants.idOfCommand, pidVelConfigData);
				for (int i = 0; i < 3; i++) {
					System.out.print("\n vp " + getVKp(i));
					System.out.print(" vd " + getVKd(i));
					System.out.println("");
				}
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		});

		addEvent(getConfig.idOfCommand, () -> {
			try {
				readFloats(getConfig.idOfCommand, pidConfigData);
				for (int i = 0; i < 3; i++) {
					System.out.print("\n p " + getKp(i));
					System.out.print(" i " + getKi(i));
					System.out.print(" d " + getKd(i));
					System.out.println("");
				}
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		});

		addEvent(pidStatus.idOfCommand, () -> {
			try {
				if (piddata == null) {
					piddata = new double[15];
					readFloats(pidStatus.idOfCommand, piddata);
				}
				readFloats(pidStatus.idOfCommand, piddata);
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		});
		addEvent(GetPIDVelocity.idOfCommand, () -> {
			try {
				if (veldata == null) {
					veldata = new double[15];
					readFloats(GetPIDVelocity.idOfCommand, veldata);
				}
				readFloats(GetPIDVelocity.idOfCommand, veldata);
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		});
		connect();
	}

	public void add2001() {
		pickOrder.waitToSendMode();
		clearFaults.waitToSendMode();
		estop.waitToSendMode();
		approve.waitToSendMode();
		for (PacketType pt : Arrays.asList(clearFaults, pickOrder, getStatus, approve, estop)) {
			addPollingPacket(pt);
		}
		addEvent(getStatus.idOfCommand, () -> {
			readBytes(getStatus.idOfCommand, status);
		});
	}

	public void addIMU() {
		addPollingPacket(IMU);
	}

	public void addIR() {
		addPollingPacket(getIR);
	}

//	public static RBE2001Robot get(String name, int myPID) throws Exception {
//		
//		return new RBE2001Robot(name, myPID);
//	}

	@Override
	public String toString() {
		return getName();
	}

	public double getNumPid() {
		return myNum;
	}

	public double getPidSetpoint(int index) {

		return pidStatus.getUpstream()[1 + index * 2 + 0].doubleValue();
	}

	public double getPidPosition(int index) {
		return pidStatus.getUpstream()[1 + index * 2 + 1].doubleValue();
	}

	/**
	 * Velocity domain values
	 * 
	 * @param index
	 * @return
	 */
	public double getHardwareOutput(int index) {
		return GetPIDVelocity.getUpstream()[1 + index * 3 + 2].doubleValue();
	}

	public double getVelocity(int index) {
		return GetPIDVelocity.getUpstream()[1 + index * 3 + 1].doubleValue();
	}

	public double getVelSetpoint(int index) {

		return GetPIDVelocity.getUpstream()[1 + index * 3 + 0].doubleValue();
	}

	public void updatConfig() {
		getConfig.oneShotMode();
		GetPDVelocityConstants.oneShotMode();
	}

	public void setPidGains(int index, double kp, double ki, double kd) {
		pidConfigData[3 * index + 0] = kp;
		pidConfigData[3 * index + 1] = ki;
		pidConfigData[3 * index + 2] = kd;
		writeFloats(setConfig.idOfCommand, pidConfigData);
		setConfig.oneShotMode();

	}

	public double getKp(int index) {
		readFloats(getConfig.idOfCommand, pidConfigData);
		return pidConfigData[(3 * index) + 0];
	}

	public double getKi(int index) {
		readFloats(getConfig.idOfCommand, pidConfigData);
		return pidConfigData[(3 * index) + 1];
	}

	public double getKd(int index) {
		readFloats(getConfig.idOfCommand, pidConfigData);
		return pidConfigData[(3 * index) + 2];
	}

	public double getVKp(int index) {
		readFloats(GetPDVelocityConstants.idOfCommand, pidVelConfigData);
		return pidVelConfigData[(3 * index) + 0];
	}

	public double getVKd(int index) {
		readFloats(GetPDVelocityConstants.idOfCommand, pidVelConfigData);
		return pidVelConfigData[(3 * index) + 2];
	}

	public void setVelocityGains(int index, double kp, double kd) {
		pidVelConfigData[3 * index + 0] = kp;
		pidVelConfigData[3 * index + 1] = 0;
		pidVelConfigData[3 * index + 2] = kd;
		writeFloats(SetPDVelocityConstants.idOfCommand, pidVelConfigData);
		SetPDVelocityConstants.oneShotMode();
	}

	public void setPidSetpoints(int msTransition, int mode, double[] data) {
		double down[] = new double[2 + getMyNumPid()];
		down[0] = msTransition;
		down[1] = mode;
		for (int i = 0; i < getMyNumPid(); i++) {
			down[2 + i] = data[i];
		}
		writeFloats(setSetpoint.idOfCommand, down);
		setSetpoint.oneShotMode();

	}

	public void setPidSetpoint(int msTransition, int mode, int index, double data) {
		double[] cur = new double[getMyNumPid()];
		for (int i = 0; i < getMyNumPid(); i++) {
			if (i == index)
				cur[index] = data;
			else
				cur[i] = getPidSetpoint(i);
		}
		cur[index] = data;
		setPidSetpoints(msTransition, mode, cur);

	}

	public void setVelocity(int index, double data) {
		double[] cur = new double[getMyNumPid()];
		for (int i = 0; i < getMyNumPid(); i++) {
			if (i == index)
				cur[index] = data;
			else
				cur[i] = getVelSetpoint(i);
		}
		cur[index] = data;
		setVelocity(cur);

	}

	public void setVelocity(double[] data) {
		writeFloats(SetPIDVelocity.idOfCommand, data);
		SetPIDVelocity.oneShotMode();

	}

	public void estop() {
		estop.oneShotMode();
	}

	public double getDriveStatus() {
		return driveStatus[0];
	}

	public void pickOrder(double material, double angle, double dropLocation) {
		pickOrderData[0] = material;
		pickOrderData[1] = angle;
		pickOrderData[2] = dropLocation;
		writeFloats(pickOrder.idOfCommand, pickOrderData);
		pickOrder.oneShotMode();

	}

	public WarehouseRobotStatus getStatus() {
		return WarehouseRobotStatus.fromValue(status[0]);
	}

	public void clearFaults() {
		clearFaults.oneShotMode();

	}

	public void approve() {
		approve.oneShotMode();

	}

	public int getMyNumPid() {
		return myNum;
	}

	public void setMyNumPid(int myNumPid) {
		if (myNumPid > 0)
			this.myNum = myNumPid;
		throw new RuntimeException("Can not have 0 PID");
	}

	public void stop(int currentIndex) {
		setPidSetpoint(0, 0, currentIndex, getPidPosition(currentIndex));
	}
}
