package radios;

public class CC2530 extends Radio{
	//433Mhz
	/*
	private static double RADIO_TX_POWER_LEVELS[] = {-28, -22, -20, -18, -16, -14, -12, -10, -8, -6, -4, -3, -1.5, -0.5, 1, 2.5, 4.5}; //dBm
	private static double RADIO_TX_POWER_CONSUMPTIONS[] = {69, 69, 72, 72, 75, 75, 75, 75, 75, 78, 78, 81, 81, 84, 93}; //mW
	private static double RADIO_RX_POWER_CONSUMPTION = 72.9; //mW
	private static double ELEC_CONST_TX = 12; //mW
	
	private static int RADIO_RECEIVER_SENSITIVITY = -97; //dBm
	private static double PLD0 = 49.46; //dBm
	private static double MAXIMUM_DISTANCE_REACHED = 400; //m
	
	private static double DATA_RATE = 250;//kbps
	*/
	
	public CC2530() {
		RADIO_TX_POWER_LEVELS = new double[17];
		RADIO_TX_POWER_LEVELS[0] = -28;
		RADIO_TX_POWER_LEVELS[1] = -22;
		RADIO_TX_POWER_LEVELS[2] = -20;
		RADIO_TX_POWER_LEVELS[3] = -18;
		RADIO_TX_POWER_LEVELS[4] = -16;
		RADIO_TX_POWER_LEVELS[5] = -14;
		RADIO_TX_POWER_LEVELS[6] = -12;
		RADIO_TX_POWER_LEVELS[7] = -10;
		RADIO_TX_POWER_LEVELS[8] = -8;
		///*
		RADIO_TX_POWER_LEVELS[9] = -6;
		RADIO_TX_POWER_LEVELS[10] = -4;
		RADIO_TX_POWER_LEVELS[11] = -3;
		RADIO_TX_POWER_LEVELS[12] = -1.5;
		RADIO_TX_POWER_LEVELS[13] = -0.5;
		RADIO_TX_POWER_LEVELS[14] = 1;
		RADIO_TX_POWER_LEVELS[15] = 2.5;
		RADIO_TX_POWER_LEVELS[16] = 4.5;
		//*/		
		RADIO_TX_POWER_CONSUMPTIONS = new double[17];
		RADIO_TX_POWER_CONSUMPTIONS[0] = 69;
		RADIO_TX_POWER_CONSUMPTIONS[1] = 69;
		RADIO_TX_POWER_CONSUMPTIONS[2] = 72;
		RADIO_TX_POWER_CONSUMPTIONS[3] = 72;
		RADIO_TX_POWER_CONSUMPTIONS[4] = 75;
		RADIO_TX_POWER_CONSUMPTIONS[5] = 75;
		RADIO_TX_POWER_CONSUMPTIONS[6] = 75;
		RADIO_TX_POWER_CONSUMPTIONS[7] = 75;
		RADIO_TX_POWER_CONSUMPTIONS[8] = 75;
		///*
		RADIO_TX_POWER_CONSUMPTIONS[9] = 78;
		RADIO_TX_POWER_CONSUMPTIONS[10] = 78;
		RADIO_TX_POWER_CONSUMPTIONS[11] = 81;
		RADIO_TX_POWER_CONSUMPTIONS[12] = 81;
		RADIO_TX_POWER_CONSUMPTIONS[13] = 84;
		RADIO_TX_POWER_CONSUMPTIONS[14] = 87;
		RADIO_TX_POWER_CONSUMPTIONS[15] = 93;
		RADIO_TX_POWER_CONSUMPTIONS[16] = 102;
		//*/
		
		RADIO_RX_POWER_CONSUMPTION = 72.9; //mW
		ELEC_CONST_TX = 12; //mW
		
		RADIO_RECEIVER_SENSITIVITY = -97; //dBm
		PLD0 = 49.46; //dBm
		MAXIMUM_DISTANCE_REACHED = 400; //m
		//MAXIMUM_DISTANCE_REACHED = 100; //m
		
		DATA_RATE = 250;//kbps
		
		ELEC_TX = 48.0;
		EPSILON = 0.00225;
		ELEC_RX = 291.6;
		
		setRadioParameters();
	}

	@Override
	public String getRadioName() {
		// TODO Auto-generated method stub
		return new String("CC2530");
	}
}
