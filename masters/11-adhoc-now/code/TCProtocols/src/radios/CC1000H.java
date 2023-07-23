package radios;

public class CC1000H extends Radio{
	//433Mhz
	/*
	private static final double RADIO_TX_POWER_LEVELS[] = {-20, -5, 0, 5, 10};
	private static final double RADIO_TX_POWER_CONSUMPTIONS[] = {15.9, 26.7, 31.2, 44.4, 80.1};
	private static final double RADIO_RX_POWER_CONSUMPTION = 22.2;
	private static final double ELEC_CONST_TX = 12; //mW
	
	private static final int RADIO_RECEIVER_SENSITIVITY = -109;
	private static final double PLD0 = 75.36;
	private static final double MAXIMUM_DISTANCE_REACHED = 152;
	
	private static final double DATA_RATE = 76;//kbps
	*/
	
	public CC1000H() {
		RADIO_TX_POWER_LEVELS = new double[5];
		RADIO_TX_POWER_LEVELS[0] = -20;
		RADIO_TX_POWER_LEVELS[1] = -5;
		RADIO_TX_POWER_LEVELS[2] = 0;
		RADIO_TX_POWER_LEVELS[3] = 5;
		RADIO_TX_POWER_LEVELS[4] = 10;
		
		RADIO_TX_POWER_CONSUMPTIONS = new double[5];
		RADIO_TX_POWER_CONSUMPTIONS[0] = 15.9;
		RADIO_TX_POWER_CONSUMPTIONS[1] = 26.7;
		RADIO_TX_POWER_CONSUMPTIONS[2] = 31.2;
		RADIO_TX_POWER_CONSUMPTIONS[3] = 44.4;
		RADIO_TX_POWER_CONSUMPTIONS[4] = 80.1;
		
		RADIO_RX_POWER_CONSUMPTION = 22.2; //mW
		ELEC_CONST_TX = 12; //mW
		
		RADIO_RECEIVER_SENSITIVITY = -109; //dBm
		PLD0 = 75.36; //dBm
		//MAXIMUM_DISTANCE_REACHED = 152; //m
		MAXIMUM_DISTANCE_REACHED = 100; //m
		
		DATA_RATE = 76;//kbps
		///*
		ELEC_TX = 48.0;
		EPSILON = 0.01608;
		ELEC_RX = 236.4;
		//*/
		setRadioParameters();
	}

	@Override
	public String getRadioName() {
		// TODO Auto-generated method stub
		return new String("CC1000H");
	}
}
