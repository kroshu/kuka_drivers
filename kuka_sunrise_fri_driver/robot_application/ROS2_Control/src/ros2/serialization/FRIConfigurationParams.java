package ros2.serialization;

import java.io.Externalizable;
import java.io.IOException;
import java.io.ObjectInput;
import java.io.ObjectOutput;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.roboticsAPI.deviceModel.Device;

public class FRIConfigurationParams implements Externalizable {

	public static final int length = 12; //remoteIP not included


	private String _remoteIP;
	private int _remotePort;
	private int _sendPeriodMilliSec;
	private int _receiveMultiplier;

	@Override
	public void writeExternal(ObjectOutput out) throws IOException {
		out.writeBytes(_remoteIP);
		out.writeInt(_remotePort);
		out.writeInt(_sendPeriodMilliSec);
		out.writeInt(_receiveMultiplier);
	}

	@Override
	public void readExternal(ObjectInput in) throws IOException,
			ClassNotFoundException {
		_remoteIP = in.readUTF();
		_remotePort = in.readInt();
		_sendPeriodMilliSec = in.readInt();
		_receiveMultiplier = in.readInt();
	}

	public FRIConfigurationParams() {
		_remoteIP = "0.0.0.0"
		_remotePort = 30200;
		_sendPeriodMilliSec = 10;
		_receiveMultiplier = 1;
	}

	public FRIConfigurationParams(FRIConfiguration friConfiguration){
		_remoteIP = friConfiguration.getHostName();
		_remotePort = friConfiguration.getPortOnRemote();
		_sendPeriodMilliSec = friConfiguration.getSendPeriodMilliSec();
		_receiveMultiplier = friConfiguration.getReceiveMultiplier();
	}

	public FRIConfiguration toFRIConfiguration(Device device){
		FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(device, _remoteIP);
		friConfiguration.setPortOnRemote(_remotePort);
		friConfiguration.setSendPeriodMilliSec(_sendPeriodMilliSec);
		friConfiguration.setReceiveMultiplier(_receiveMultiplier);
		return friConfiguration;
	}
}
