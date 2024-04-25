package ros2.serialization;

import java.io.Externalizable;
import java.io.IOException;
import java.io.ObjectInput;
import java.io.ObjectOutput;
import java.util.Arrays;


import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

public abstract class ControlModeParams implements Externalizable{
	public static int length = 0;

	private enum ControlModeID{
		POSITION(		(byte)1),
		JOINT_IMPEDANCE((byte)2);

		public final byte value;

		ControlModeID(byte value){
			this.value = value;
		}

		public static ControlModeID fromByte(byte value){
			for(ControlModeID id : ControlModeID.values()){
				if(value == id.value){
					return id;
				}
			}
			throw new RuntimeException("Byte " + value + " does not represent an ControlModeID");
		}
	}

	public static ControlModeParams fromSerialData(byte[] serialData){
		if(serialData.length == 0){
			throw new RuntimeException("serialData is empty");
		}
		ControlModeID controlModeID = ControlModeID.fromByte(serialData[0]);
		ControlModeParams controlModeParams = null;
		switch(controlModeID){
			case POSITION:
				controlModeParams = new PositionControlModeParams();
				break;
			case JOINT_IMPEDANCE:
				controlModeParams = new JointImpedanceControlModeParams();
				break;
		}
		serialData = Arrays.copyOfRange(serialData, 1, serialData.length);
		MessageEncoding.Decode(serialData, controlModeParams);
		return controlModeParams;
	}

	public static ControlModeParams fromMotionControlMode(IMotionControlMode controlMode){
		if(controlMode == null){
			throw new RuntimeException("ControlMode is null");
		}
		ControlModeParams controlModeParams;
		if(controlMode instanceof PositionControlMode){
			controlModeParams = new PositionControlModeParams();
		} else if (controlMode instanceof JointImpedanceControlMode){
			controlModeParams = new JointImpedanceControlModeParams((JointImpedanceControlMode) controlMode);
		} else {
			throw new RuntimeException("Control mode not supported");
		}
		return controlModeParams;
	}

	@Override
	public void writeExternal(ObjectOutput out) throws IOException {


	}

	@Override
	public void readExternal(ObjectInput in) throws IOException,
			ClassNotFoundException {

	}

}

class PositionControlModeParams extends ControlModeParams{


}

class JointImpedanceControlModeParams extends ControlModeParams{
	public JointImpedanceControlModeParams(){

	}
	public JointImpedanceControlModeParams(JointImpedanceControlMode controlMode){

	}
}
