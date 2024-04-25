package ros2.modules;

import java.io.Externalizable;
import java.util.Arrays;

import ros2.modules.FRIManager;
import ros2.serialization.ControlModeParams;
import ros2.serialization.FRIConfigurationParams;
import ros2.serialization.JointImpedanceControlModeExternalizable;
import ros2.serialization.MessageEncoding;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

public class ROS2Connection {

	private TCPConnection _TCPConnection;
	private FRIManager _FRIManager;
	private boolean _acceptingCommands = false;
	private boolean _disconnect = false;

	public void registerTCPConnectionModule(TCPConnection tcpConnectionModule){
		_TCPConnection = tcpConnectionModule;
	}

	public void registerFRIManagerModule(FRIManager friManagerModule){
		_FRIManager = friManagerModule;
	}

	public void acceptCommands(){
		_acceptingCommands = true;
	}

	public void rejectCommands(){
		_acceptingCommands = false;
	}

	private enum CommandState{
		ACCEPTED((byte)1),
		REJECTED((byte)2),
		UNKNOWN((byte)3),
		ERROR_CONTROL_ENDED((byte)4),
		ERROR_FRI_ENDED((byte)5);

		private final byte value;

		private CommandState(byte value){
			this.value = value;
		}
	}

	private enum CommandID{
		CONNECT(			(byte)1),
		DISCONNECT(			(byte)2),
		START_FRI(			(byte)3),
		END_FRI(			(byte)4),
		ACTIVATE_CONTROL(	(byte)5),
		DEACTIVATE_CONTROL(	(byte)6),
		GET_FRI_CONFIG(		(byte)7),
		SET_FRI_CONFIG(		(byte)8),
		GET_CONTROL_MODE(	(byte)9),
		SET_CONTROL_MODE(	(byte)10),
		GET_COMMAND_MODE(	(byte)11),
		SET_COMMAND_MODE(	(byte)12);

		private final byte value;

		private CommandID(byte value){
			this.value = value;
		}


		public static CommandID fromByte(byte value){
			for(CommandID id : CommandID.values()){
				if(value == id.value){
					return id;
				}
			}
			throw new RuntimeException("Byte " + value + " does not represent an InMessageID");
		}
	}

	private enum SuccessSignalID {
		SUCCESS((byte)1),
		NO_SUCCESS((byte)2);

		private final byte value;

		private SuccessSignalID(byte value){
			this.value = value;
		}
	}

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
			throw new RuntimeException("Byte " + value + " does not represent a ControlModeID");
		}
	}

	public void handleMessageFromROS(byte[] inMessage){
		CommandID command = null;
		try{
			ensureArrayLength(inMessage, 1);
			command = CommandID.fromByte(inMessage[0]);
			if(!_acceptingCommands){
				feedbackCommandRejected(command, new String("Not accepting commands").getBytes());
				return;
			}

		} catch(RuntimeException e){
			System.out.println(e.getMessage());
			feedbackCommandUnknown(e.getMessage().getBytes());
			return;
		}

		try{
			byte[] inMessageData = Arrays.copyOfRange(inMessage, 1, inMessage.length);
			byte[] feedbackData = null;
			System.out.println("Command received: " + command.toString());

			switch(command){
				case CONNECT:
					feedbackData = connect(inMessageData);
					break;
				case DISCONNECT:
					feedbackData = disconnect(inMessageData);
					break;
				case START_FRI:
					feedbackData = startFRI(inMessageData);
					break;
				case END_FRI:
					feedbackData = endFRI(inMessageData);
					break;
				case ACTIVATE_CONTROL:
					feedbackData = activateControl(inMessageData);
					break;
				case DEACTIVATE_CONTROL:
					feedbackData = deactivateControl(inMessageData);
					break;
				case GET_FRI_CONFIG:
					feedbackData = getFRIConfig(inMessageData);
					break;
				case SET_FRI_CONFIG:
					feedbackData = setFRIConfig(inMessageData);
					break;
				case GET_CONTROL_MODE:
					feedbackData = getControlMode(inMessageData);
					break;
				case SET_CONTROL_MODE:
					feedbackData = setControlMode(inMessageData);
					break;
				case GET_COMMAND_MODE:
					feedbackData = getCommandMode(inMessageData);
					break;
				case SET_COMMAND_MODE:
					feedbackData = setCommandMode(inMessageData);
					break;
			}
			System.out.println("Command executed.");
			feedbackCommandSuccess(command, feedbackData);
		} catch(RuntimeException e){
			System.out.println(e.getMessage());
			feedbackCommandNoSuccess(command, e.getMessage().getBytes());
			return;
		}
	}

	public void handleConnectionLost(){
		_FRIManager.deactivateControl();
		_FRIManager.endFRI();
		_FRIManager.close();
		System.out.println("Error: connection lost. FRI ended.");
	}

	public void handleControlEndedError(){
		byte[] message = {CommandState.ERROR_CONTROL_ENDED.value};
		System.out.println("Error: control ended");
		_TCPConnection.sendBytes(message);
	}

	public void handleFRIEndedError(){
		byte[] message = {CommandState.ERROR_FRI_ENDED.value};
		System.out.println("Error: session ended");
		_TCPConnection.sendBytes(message);
	}

	private void feedbackCommandRejected(CommandID command, byte[] feedbackData){
		byte[] message = appendByte(command.value, feedbackData);
		message = appendByte(CommandState.REJECTED.value, message);
		_TCPConnection.sendBytes(message);
	}

	private void feedbackCommandUnknown(byte[] feedbackData){
		byte[] message = appendByte(CommandState.UNKNOWN.value, feedbackData);
		_TCPConnection.sendBytes(message);
	}

	private void feedbackCommandSuccess(CommandID command, byte[] feedbackData){
		byte[] message = appendByte(SuccessSignalID.SUCCESS.value, feedbackData);
		message = appendByte(command.value, message);
		message = appendByte(CommandState.ACCEPTED.value, message);
		_TCPConnection.sendBytes(message);
	}

	private void feedbackCommandNoSuccess(CommandID command, byte[] feedbackData){
		byte[] message = appendByte(SuccessSignalID.NO_SUCCESS.value, feedbackData);
		message = appendByte(command.value, message);
		message = appendByte(CommandState.ACCEPTED.value, message);
		_TCPConnection.sendBytes(message);
	}

	private byte[] appendByte(byte id, byte[] data){
		byte[] message = null;
		if(data == null){
			message = new byte[]{id};
		} else {
			message = new byte[data.length + 1];
			message[0] = id;
			System.arraycopy(data, 0, message, 1, data.length);
		}
		return message;
	}

	private byte[] connect(byte[] cmdData){
		return null;
	}

	private byte[] disconnect(byte[] cmdData){
		_FRIManager.close();
		_disconnect = true;
		//_TCPConnection.closeConnection(); //TODO: close connection after feedback was sent
		return null;
	}

	private byte[] startFRI(byte[] cmdData){
		FRIManager.CommandResult commandResult = _FRIManager.startFRI();
		if(commandResult == FRIManager.CommandResult.REJECTED){
			throw new RuntimeException("Command rejected");
		}
		if(commandResult == FRIManager.CommandResult.ERRORED){
			throw new RuntimeException("Command errored");
		}
		return null;
	}

	private byte[] endFRI(byte[] cmdData){
		FRIManager.CommandResult commandResult = _FRIManager.endFRI();
		if(commandResult == FRIManager.CommandResult.REJECTED){
			throw new RuntimeException("Command rejected");
		}
		if(commandResult == FRIManager.CommandResult.ERRORED){
			throw new RuntimeException("Command errored");
		}
		return null;
	}

	private byte[] activateControl(byte[] cmdData){
		FRIManager.CommandResult commandResult = _FRIManager.activateControl();
		if(commandResult == FRIManager.CommandResult.REJECTED){
			throw new RuntimeException("Command rejected");
		}
		if(commandResult == FRIManager.CommandResult.ERRORED){
			throw new RuntimeException("Command errored");
		}
		return null;
	}

	private byte[] deactivateControl(byte[] cmdData){
		FRIManager.CommandResult commandResult = _FRIManager.deactivateControl();
		if(commandResult == FRIManager.CommandResult.REJECTED){
			throw new RuntimeException("Command rejected");
		}
		if(commandResult == FRIManager.CommandResult.ERRORED){
			throw new RuntimeException("Command errored");
		}
		return null;
	}

	private byte[] getFRIConfig(byte[] cmdData){
		FRIConfigurationParams friConfigurationParams = _FRIManager.getFRIConfig();
		byte[] feedbackData = MessageEncoding.Encode(friConfigurationParams, FRIConfigurationParams.length);
		return feedbackData;
	}

	private byte[] setFRIConfig(byte[] cmdData) {
		ensureArrayLength(cmdData, FRIConfigurationParams.length + 6);

		FRIConfigurationParams friConfigurationParams = new FRIConfigurationParams();
		MessageEncoding.Decode(cmdData, friConfigurationParams);
		FRIManager.CommandResult commandResult = _FRIManager.setFRIConfig(friConfigurationParams);
		if(commandResult == FRIManager.CommandResult.REJECTED){
			throw new RuntimeException("Command rejected");
		}
		if(commandResult == FRIManager.CommandResult.ERRORED){
			throw new RuntimeException("Command errored");
		}
		return null;
	}

	private byte[] getControlMode(byte[] cmdData){
		IMotionControlMode controlMode = _FRIManager.getControlMode();
		ControlModeID controlModeID;
		byte[] controlModeData;
		if(controlMode instanceof PositionControlMode){
			controlModeID = ControlModeID.POSITION;
			controlModeData = null;
		} else if (controlMode instanceof JointImpedanceControlMode){
			controlModeID = ControlModeID.JOINT_IMPEDANCE;
			controlModeData = MessageEncoding.Encode(new JointImpedanceControlModeExternalizable((JointImpedanceControlMode)controlMode), JointImpedanceControlModeExternalizable.length);
		} else {
			throw new RuntimeException("Control mode not supported");
		}
		byte[] message = appendByte(controlModeID.value, controlModeData);
		return message;
	}

	private byte[] setControlMode(byte[] cmdData){
		ensureArrayLength(cmdData, 1);
		ControlModeID controlModeID = ControlModeID.fromByte(cmdData[0]);

		byte[] controlModeData = Arrays.copyOfRange(cmdData, 1, cmdData.length);

		IMotionControlMode controlMode = null;
		switch(controlModeID){
			case POSITION:
				controlMode = new PositionControlMode();
				break;
			case JOINT_IMPEDANCE:
				ensureArrayLength(controlModeData, JointImpedanceControlModeExternalizable.length + 6);
				JointImpedanceControlModeExternalizable externalizable = new JointImpedanceControlModeExternalizable();
				System.out.println("Decoding params");
				MessageEncoding.Decode(controlModeData, externalizable);
				controlMode = externalizable.toControlMode();
				break;
		}
		System.out.println("Control mode decoded.");
		FRIManager.CommandResult commandResult = _FRIManager.setControlMode(controlMode);
		if(commandResult == FRIManager.CommandResult.REJECTED){
			throw new RuntimeException("Command rejected");
		}
		if(commandResult == FRIManager.CommandResult.ERRORED){
			throw new RuntimeException("Command errored");
		}
		return null;
	}

	private byte[] getCommandMode(byte[] cmdData){
		ClientCommandMode clientCommandMode = _FRIManager.getClientCommandMode();
		byte[] commandModeData = new byte[1];
		commandModeData[0] = (byte)clientCommandMode.ordinal();//TODO: check if ordinal == value
		return commandModeData;
	}

	private byte[] setCommandMode(byte[] cmdData){
		ensureArrayLength(cmdData, 1);
		ClientCommandMode clientCommandMode = ClientCommandMode.intToVal((int)cmdData[0]);
		if(clientCommandMode == ClientCommandMode.NO_COMMAND_MODE){
			throw new RuntimeException("Byte " + cmdData[0] + " does not represent a ClientCommandMode.");
		}
		FRIManager.CommandResult commandResult = _FRIManager.setCommandMode(clientCommandMode);
		if(commandResult == FRIManager.CommandResult.REJECTED){
			throw new RuntimeException("Command rejected");
		}
		if(commandResult == FRIManager.CommandResult.ERRORED){
			throw new RuntimeException("Command errored");
		}
		return null;
	}

	private void ensureArrayLength(byte[] array, int requiredLength){
		if(array == null){
			throw new RuntimeException("Array is null");
		}
		if(array.length < requiredLength){
			throw new RuntimeException("Array does not satisfy length requirement. Required length: " + requiredLength + ", actual length: " + array.length);
		}
	}

}
