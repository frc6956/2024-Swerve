package frc.robot;

/**
 * Contains the definitions of all the ports
 */
public class Ports {

		// IP (v4) addresses
		// The purpose of this section is to serve as a reminder of what static IP (v4) addresses are used so they are consistent
		// between the competition and practice robots.
		//
		// The radio is automatically set to 10.24.95.1
		// The Rio is set to static 10.24.95.2, mask 255.255.255.0
		// The Limelight is set to 10.24.95.11, mask 255.255.255.0, gateway 10.24.95.1
		// but note that pressing the reset button will revert to DHCP.
		// The Raspberry Pi running FRCVision is set to static 10.24.95.12, mask 255.255.255.0, gateway 10.24.95.1, DNS blank
		//
		// If a device cannot be accessed (e.g. because its address was somehow obtained via DHCP and mDNS is not working),
		// use Angry IP Scanner to find it!


		/**
		 * Analog ports
		 */
		public static class Analog {
			//public static final int SONAR = 3;
			//public static final int PRESSURE_SENSOR = 1;

			// 2023 Off-season
			// SPARK MAX Absolute encoders
			
			public static final int FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER = 0;
			public static final int FRONT_LEFT_TURNING_ABSOLUTE_ENCODER = 1;
            public static final int REAR_LEFT_TURNING_ABSOLUTE_ENCODER = 2;
            public static final int REAR_RIGHT_TURNING_ABSOLUTE_ENCODER = 3;
			public static final int Pigeon = 0;
		}
		
		/**
		 * CAN Ids
		 */
		public static class CAN {
			// 2019 robot
			/*
			public static final int RIGHT_FRONT = 1;
			public static final int RIGHT_REAR = 2;
			public static final int LEFT_FRONT = 3;
			public static final int LEFT_REAR = 4;
			public static final int FRONT_CENTER = 5; // unused
			public static final int REAR_CENTER = 6; // unused
			public static final int ELEVATOR = 7;
			public static final int SHOOTER_LEFT = 8;
			public static final int SHOOTER_RIGHT = 9;
			public static final int HINGE = 10;
			public static final int WINCH = 11; 
			public static final int HAB_ELEVATOR = 12;
			public static final int SPINNER = 13;
			public static final int PCM = 9;
			public static final int PDP = 0;
			*/


			//2023 Off-season
			public static final int PCM = 1;
			public static final int PDP = 0;	

			// SPARK MAX CAN IDs
			public static final int FRONT_LEFT_DRIVING = 4;
			public static final int REAR_LEFT_DRIVING = 2;
			public static final int FRONT_RIGHT_DRIVING = 5;
			public static final int REAR_RIGHT_DRIVING = 8;

			public static final int FRONT_LEFT_TURNING = 3;
			public static final int REAR_LEFT_TURNING = 1;
			public static final int FRONT_RIGHT_TURNING = 6;
			public static final int REAR_RIGHT_TURNING = 7;
		}
		
		/**
		 * USB ports
		 */
		public static class USB {
			//public static final int RIGHT_JOYSTICK = 0;
			//public static final int LEFT_JOYSTICK = 1;
			public static final int DRIVER_GAMEPAD = 0;
			//public static final int COPILOT_GAMEPAD = 2;
			//public static final int MAIN_JOYSTICK = 4;
		}
		
		/**
		 * PCM ports
		 */
		public static class PCM {
			// 2019 robot
			/*public static final int KICKER_OUT = 0;
			public static final int KICKER_IN = 1;		
			public static final int SUCKER_EXHALE = 2;
			public static final int SUCKER_INHALE= 3;
			public static final int EJECTOR_RETRACTED = 5;
			public static final int EJECTOR_EXTENDED = 4;			
			public static final int HOOK_UP = 7;
			public static final int HOOK_DOWN = 6;*/

		}
	}