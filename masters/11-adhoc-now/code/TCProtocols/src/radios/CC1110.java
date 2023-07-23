package radios;

public class CC1110 extends Radio{
	//433Mhz
	/*
	private static final double RADIO_TX_POWER_LEVELS[] = {-30, -20, -15, -10, -5, 0, 5, 7, 10};
	private static final double RADIO_TX_POWER_CONSUMPTIONS[] = {35.1, 36.6, 38.4, 41.4, 42.9, 48.3, 57.9, 72, 86.4};
	private static final double RADIO_RX_POWER_CONSUMPTION = 66;
	private static final double ELEC_CONST_TX = 12; //mW
	
	private static final int RADIO_RECEIVER_SENSITIVITY = -99;
	private static final double PLD0 = 75.36;
	private static final double MAXIMUM_DISTANCE_REACHED = 152;
	
	private static final double DATA_RATE = 500;//kbps
	*/
	
	public CC1110 () {
		RADIO_TX_POWER_LEVELS = new double[9];
		RADIO_TX_POWER_LEVELS[0] = -30;
		RADIO_TX_POWER_LEVELS[1] = -20;
		RADIO_TX_POWER_LEVELS[2] = -15;
		RADIO_TX_POWER_LEVELS[3] = -10;
		RADIO_TX_POWER_LEVELS[4] = -5;
		RADIO_TX_POWER_LEVELS[5] =0;
		RADIO_TX_POWER_LEVELS[6] = 5;
		RADIO_TX_POWER_LEVELS[7] = 7;
		RADIO_TX_POWER_LEVELS[8] = 10;
		//{35.1, 36.6, 38.4, 41.4, 42.9, 48.3, 57.9, 72, 86.4};
		RADIO_TX_POWER_CONSUMPTIONS = new double[9];
		RADIO_TX_POWER_CONSUMPTIONS[0] = 35.1;
		RADIO_TX_POWER_CONSUMPTIONS[1] = 36.6;
		RADIO_TX_POWER_CONSUMPTIONS[2] = 38.4;
		RADIO_TX_POWER_CONSUMPTIONS[3] = 41.4;
		RADIO_TX_POWER_CONSUMPTIONS[4] = 42.9;
		RADIO_TX_POWER_CONSUMPTIONS[5] = 48.3;
		RADIO_TX_POWER_CONSUMPTIONS[6] = 57.9;
		RADIO_TX_POWER_CONSUMPTIONS[7] = 72;
		RADIO_TX_POWER_CONSUMPTIONS[8] = 86.4;		
		
		RADIO_RX_POWER_CONSUMPTION = 66; //mW
		ELEC_CONST_TX = 12; //mW
		
		RADIO_RECEIVER_SENSITIVITY = -99; //dBm
		//PLD0 = 75.36; //dBm
		MAXIMUM_DISTANCE_REACHED = 152; //m
		
		DATA_RATE = 500;//kbps
		
		setRadioParameters();
	}
	@Override
	public double getElecRx() {
		return Radio.ELEC_RX;
		// TODO Auto-generated method stub
	}
	@Override
	public double getElecTx() {
		return Radio.ELEC_TX;
		// TODO Auto-generated method stub
	}
	@Override
	public double getEpsilon() {
		return EPSILON;
		// TODO Auto-generated method stub
	}
	@Override
	public double getMaximumDistanceReached() {
		return MAXIMUM_DISTANCE_REACHED;
		// TODO Auto-generated method stub
	}
	@Override
	public double getPLD0() {
		return PLD0;
		// TODO Auto-generated method stub
	}
	@Override
	public int getRadioReceiverSensitivity() {
		return RADIO_RECEIVER_SENSITIVITY;
		// TODO Auto-generated method stub
	}
	@Override
	public double getRadioRxPowerConsumption() {
		return RADIO_RX_POWER_CONSUMPTION;
		// TODO Auto-generated method stub
	}
	@Override
	public double[] getRadioTxPowerConsumptions() {
		return RADIO_TX_POWER_CONSUMPTIONS;
		// TODO Auto-generated method stub
	}
	@Override
	public double[] getRadioTxPowerLevels() {
		return RADIO_TX_POWER_LEVELS;
		// TODO Auto-generated method stub
	}
	@Override
	public String getRadioName() {
		// TODO Auto-generated method stub
		return new String("CC2420");
	}
}
