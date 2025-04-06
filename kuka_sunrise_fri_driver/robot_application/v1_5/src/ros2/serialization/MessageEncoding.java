package ros2.serialization;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.Externalizable;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.ObjectInputStream;

public class MessageEncoding {
	public static byte[] Encode(Externalizable objectIn, int serialLength){
		ByteArrayOutputStream serialDataStream = new ByteArrayOutputStream();
		byte[] serialDataOut = new byte[serialLength + 4];
		try{
			ObjectOutputStream objectStream = new ObjectOutputStream(serialDataStream);
			objectIn.writeExternal(objectStream);
			objectStream.flush();
			objectStream.close();
			serialDataOut = serialDataStream.toByteArray();
		}catch(IOException e){
			e.printStackTrace();
		}
		return serialDataOut;
	}

	public static void Decode(byte[] serialDataIn, Externalizable objectOut) throws RuntimeException{
		try{
			ByteArrayInputStream serialDataStream = new ByteArrayInputStream(serialDataIn);
			ObjectInputStream objectStream	= new ObjectInputStream(serialDataStream);
			objectOut.readExternal(objectStream);
			objectStream.close();
		} catch(IOException e){
			e.printStackTrace();
			throw new RuntimeException("IO Exception occurred");
		} catch (ClassNotFoundException e) {
			throw new RuntimeException("Message could not be decoded");
		}
	}
}
