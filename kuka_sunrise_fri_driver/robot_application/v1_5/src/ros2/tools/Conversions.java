package ros2.tools;

public class Conversions {
	public static class Arrays{
		public static float[] DoubleToFloat(double[] doubleArray){
			float[] floatArray = new float[doubleArray.length];
			for(int i = 0; i < doubleArray.length; i++){
				floatArray[i] = (float)doubleArray[i];
			}
			return floatArray;
		}
	}
}
