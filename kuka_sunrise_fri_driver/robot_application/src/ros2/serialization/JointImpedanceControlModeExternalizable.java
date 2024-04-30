package ros2.serialization;

import java.io.Externalizable;
import java.io.IOException;
import java.io.ObjectInput;
import java.io.ObjectOutput;

import ros2.tools.Conversions;

import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

public class JointImpedanceControlModeExternalizable extends JointImpedanceControlMode implements Externalizable{

	public final static int length = 112;

	public JointImpedanceControlModeExternalizable(){
		super(1000, 1000, 1000, 1000, 1000, 1000, 1000);
	}

	public JointImpedanceControlModeExternalizable(JointImpedanceControlMode other){
		super(other);
	}

	public IMotionControlMode toControlMode(){
		JointImpedanceControlMode controlMode = new JointImpedanceControlMode((JointImpedanceControlMode)this);
		return (IMotionControlMode)controlMode;
	}

	@Override
	public void writeExternal(ObjectOutput out) throws IOException {
		for(double jointStiffness : getStiffness()){
			out.writeDouble(jointStiffness);
		}
		for(double jointDamping : getDamping()){
			out.writeDouble(jointDamping);
		}
	}

	@Override
	public void readExternal(ObjectInput in) throws IOException,
			ClassNotFoundException {
		double[] jointStiffness = new double[getStiffness().length];
		for(int i = 0; i < getStiffness().length; i++){
			jointStiffness[i] = in.readDouble();
		}
		setStiffness(jointStiffness);

		double[] jointDamping = new double[getDamping().length];
		for(int i = 0; i < getDamping().length; i++){
			jointDamping[i] = in.readDouble();
		}
		setDamping(jointDamping);

	}



}
