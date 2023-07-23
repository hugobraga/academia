package radios;

public class RF230 extends Radio{
	/*
	private static final double RADIO_TX_POWER_LEVELS[] = {-17, -3, 3};
	private static final double RADIO_TX_POWER_CONSUMPTIONS[] = {30, 39, 51};
	private static final double RADIO_RX_POWER_CONSUMPTION = 48;
	private static final double ELEC_CONST_TX = 12; //mW
	
	private static final int RADIO_RECEIVER_SENSITIVITY = -101;
	private static final double PLD0 = 54.45;
	private static final double MAXIMUM_DISTANCE_REACHED = 300;
	
	private static final double DATA_RATE = 250;//kbps
	*/
	
	public RF230() {
		RADIO_TX_POWER_LEVELS = new double[3];
		RADIO_TX_POWER_LEVELS[0] = -17;
		RADIO_TX_POWER_LEVELS[1] = -3;
		RADIO_TX_POWER_LEVELS[2] = 3;
		
		RADIO_TX_POWER_CONSUMPTIONS = new double[3];
		RADIO_TX_POWER_CONSUMPTIONS[0] = 30;
		RADIO_TX_POWER_CONSUMPTIONS[1] = 39;
		RADIO_TX_POWER_CONSUMPTIONS[2] = 51;
		
		RADIO_RX_POWER_CONSUMPTION = 48; //mW
		ELEC_CONST_TX = 12; //mW
		
		RADIO_RECEIVER_SENSITIVITY = -101; //dBm
		PLD0 = 54.45; //dBm
		MAXIMUM_DISTANCE_REACHED = 300; //m
		//MAXIMUM_DISTANCE_REACHED = 100; //m
		
		DATA_RATE = 250;//kbps
		
		ELEC_TX = 48.0;
		EPSILON = 0.0017333333333333333;
		ELEC_RX = 192.0;
		
		setRadioParameters();
	}

	@Override
	public String getRadioName() {
		// TODO Auto-generated method stub
		return new String("RF230");
	}
}
