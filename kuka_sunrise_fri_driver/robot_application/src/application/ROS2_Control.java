package application;

import javax.inject.Inject;

import ros2.modules.FRIManager;
import ros2.modules.ROS2Connection;
import ros2.modules.TCPConnection;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a
 * {@link RoboticsAPITask#run()} method, which will be called successively in
 * the application lifecycle. The application will terminate automatically after
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an
 * exception is thrown during initialization or run.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the
 * {@link RoboticsAPITask#dispose()} method.</b>
 *
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class ROS2_Control extends RoboticsAPIApplication {
	@Inject
	private LBR _lbr;

	private TCPConnection _TCPConnection;
	private ROS2Connection _ROS2Connection;
	private FRIManager _FRIManager;

	@Override
	public void initialize() {
		// initialize your application here
		_TCPConnection = new TCPConnection(30000);
		_ROS2Connection = new ROS2Connection();
		_FRIManager = new FRIManager(_lbr, getApplicationControl());

		_FRIManager.registerROS2ConnectionModule(_ROS2Connection);
		_TCPConnection.registerROS2ConnectionModule(_ROS2Connection);
		_ROS2Connection.registerTCPConnectionModule(_TCPConnection);
		_ROS2Connection.registerFRIManagerModule(_FRIManager);
	}

	@Override
	public void run() {
		// your application execution starts here
		_ROS2Connection.acceptCommands();
		_TCPConnection.openConnection();
		_TCPConnection.waitUntilDisconnected();
		_ROS2Connection.rejectCommands();
		_FRIManager.close();
	}

	@Override
	public void dispose() {
		getLogger().info("disposes");
		_TCPConnection.closeConnection();
		_FRIManager.close();
	}
}
