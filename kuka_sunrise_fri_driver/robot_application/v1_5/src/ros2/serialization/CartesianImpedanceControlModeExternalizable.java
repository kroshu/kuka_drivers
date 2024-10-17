package ros2.serialization;

import java.io.Externalizable;
import java.io.IOException;
import java.io.ObjectInput;
import java.io.ObjectOutput;

import ros2.tools.Conversions;

import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

public class CartesianImpedanceControlModeExternalizable extends CartesianImpedanceControlMode implements Externalizable{

	public final static int length = 96;


	public CartesianImpedanceControlModeExternalizable(CartesianImpedanceControlMode other){
		super(other);
	}

	public CartesianImpedanceControlModeExternalizable(){
		super();
	}

	public IMotionControlMode toControlMode(){
		CartesianImpedanceControlMode controlMode = new CartesianImpedanceControlMode((CartesianImpedanceControlMode)this);
		return (IMotionControlMode)controlMode;
	}

	@Override
	public void writeExternal(ObjectOutput out) throws IOException {
		for(double cartStiffness : getStiffness()){
			out.writeDouble(cartStiffness);
		}
		for(double cartDamping : getDamping()){
			out.writeDouble(cartDamping);
		}
	}

	@Override
	public void readExternal(ObjectInput in) throws IOException,
			ClassNotFoundException {
		double[] cartStiffness = new double[getStiffness().length];
		for(int i = 0; i < getStiffness().length; i++){
			cartStiffness[i] = in.readDouble();
		}
		this.parametrize(CartDOF.X).setStiffness(cartStiffness[0]);
		this.parametrize(CartDOF.Y).setStiffness(cartStiffness[1]);
		this.parametrize(CartDOF.Z).setStiffness(cartStiffness[2]);
		this.parametrize(CartDOF.A).setStiffness(cartStiffness[3]);
		this.parametrize(CartDOF.B).setStiffness(cartStiffness[4]);
		this.parametrize(CartDOF.C).setStiffness(cartStiffness[5]);

		double[] cartDamping = new double[getDamping().length];
		for(int i = 0; i < getDamping().length; i++){
			cartDamping[i] = in.readDouble();
		}
		this.parametrize(CartDOF.X).setDamping(cartDamping[0]);
		this.parametrize(CartDOF.Y).setDamping(cartDamping[1]);
		this.parametrize(CartDOF.Z).setDamping(cartDamping[2]);
		this.parametrize(CartDOF.A).setDamping(cartDamping[3]);
		this.parametrize(CartDOF.B).setDamping(cartDamping[4]);
		this.parametrize(CartDOF.C).setDamping(cartDamping[5]);

	}



}
