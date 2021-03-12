package edu.wpi.rbe.rbe2001.fieldsimulator.robot;

import edu.wpi.SimplePacketComs.BytePacketType;
import edu.wpi.SimplePacketComs.PacketType;
import edu.wpi.SimplePacketComs.device.UdpDevice;

public class HALDevice extends UdpDevice {
	PacketType allpacket = new BytePacketType(1962, 64);

	public HALDevice(String name) throws Exception {
		super(name);
		allpacket.waitToSendMode();
		addPollingPacket(allpacket);
	}
	public void send() {
		writeBytes(1962,new byte[] {5,7});
		allpacket.oneShotMode();
	}

}
