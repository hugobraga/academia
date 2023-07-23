package radios;

public class CC2420H extends Radio{
	//433Mhz
	/*
	private static double RADIO_TX_POWER_LEVELS[] = {-25, -15, -10, -5, 0}; //dBm
	private static final double RADIO_TX_POWER_CONSUMPTIONS[] = {25.5, 29.7, 33.6, 42, 52.2}; //mW
	private static final double RADIO_RX_POWER_CONSUMPTION = 59.1; //mW
	private static final double ELEC_CONST_TX = 12; //mW
	
	private static final int RADIO_RECEIVER_SENSITIVITY = -94; //dBm
	private static final double PLD0 = 54; //dBm
	private static final double MAXIMUM_DISTANCE_REACHED = 100; //m
	
	private static final double DATA_RATE = 250;//kbps
	*/
	
	public CC2420H() {
		RADIO_TX_POWER_LEVELS = new double[5];
		RADIO_TX_POWER_LEVELS[0] = -25;
		RADIO_TX_POWER_LEVELS[1] = -15;
		RADIO_TX_POWER_LEVELS[2] = -10;
		RADIO_TX_POWER_LEVELS[3] = -5;
		RADIO_TX_POWER_LEVELS[4] = 0;
		
		RADIO_TX_POWER_CONSUMPTIONS = new double[5];
		RADIO_TX_POWER_CONSUMPTIONS[0] = 25.5;
		RADIO_TX_POWER_CONSUMPTIONS[1] = 29.7;
		RADIO_TX_POWER_CONSUMPTIONS[2] = 33.6;
		RADIO_TX_POWER_CONSUMPTIONS[3] = 42;
		RADIO_TX_POWER_CONSUMPTIONS[4] = 52.2;
		
		RADIO_RX_POWER_CONSUMPTION = 59.1; //mW
		ELEC_CONST_TX = 12; //mW
		
		RADIO_RECEIVER_SENSITIVITY = -94; //dBm
		PLD0 = 54; //dBm
		//MAXIMUM_DISTANCE_REACHED = 100; //m
		MAXIMUM_DISTANCE_REACHED = 100; //m
		
		DATA_RATE = 250;//kbps
		
		ELEC_TX = 157.89473684210526;
		EPSILON = 0.038783441463770225;
		ELEC_RX = 292.1052631578947;
		
		setRadioParameters();
	}

	@Override
	public String getRadioName() {
		// TODO Auto-generated method stub
		return new String("CC2420H");
	}
}
