package radios;

public abstract class Radio {
	protected static double RADIO_TX_POWER_LEVELS[]; //dBm
	protected static double RADIO_TX_POWER_CONSUMPTIONS[]; //mW
	protected static double RADIO_RX_POWER_CONSUMPTION; //mW
	protected static double ELEC_CONST_TX; //mW
	
	protected static int RADIO_RECEIVER_SENSITIVITY; //dBm
	protected static double PLD0; //dBm
	protected static double MAXIMUM_DISTANCE_REACHED; //m
	
	protected static double DATA_RATE;//kbps	
	
	protected static double ELEC_TX; //nJ/bit
	protected static double ELEC_RX; //nJ/bit
	protected static double EPSILON; //nJ/bit/m2	
	
	//protected void setRadioParameters(double ELEC_CONST_TX, double DATA_RATE, double MAX_TX_POWER_CONSUMPTIONS, double RADIO_RX_POWER_CONSUMPTION, double MAXIMUM_DISTANCE_REACHED) {
	protected void setRadioParameters() {
		/*
		ELEC_TX = (ELEC_CONST_TX/DATA_RATE)*Math.pow(10, 3);
		ELEC_RX = (RADIO_RX_POWER_CONSUMPTION/DATA_RATE)*Math.pow(10, 3);
		double totalElecTx = (RADIO_TX_POWER_CONSUMPTIONS[RADIO_TX_POWER_CONSUMPTIONS.length-1]/DATA_RATE)*Math.pow(10, 3);
		EPSILON = (totalElecTx - ELEC_TX)/Math.pow(MAXIMUM_DISTANCE_REACHED, RadioParameters.npathLossExp);
		System.out.println("constantes do radio: "+ELEC_TX+" "+EPSILON+" "+ELEC_RX);
		*/
		
		//PLD0 = (RADIO_TX_POWER_LEVELS[RADIO_TX_POWER_LEVELS.length-1] - (int)Math.ceil(RADIO_RECEIVER_SENSITIVITY) - 10*RadioParameters.npathLossExp*Math.log10(MAXIMUM_DISTANCE_REACHED));
		//System.out.println("PLD0: "+PLD0);
	}
	
	public double getElecRx() {
		return Radio.ELEC_RX;
		// TODO Auto-generated method stub
	}

	public double getElecTx() {
		return Radio.ELEC_TX;
		// TODO Auto-generated method stub
	}

	public double getEpsilon() {
		return EPSILON;
		// TODO Auto-generated method stub
	}

	public double getMaximumDistanceReached() {
		return MAXIMUM_DISTANCE_REACHED;
		// TODO Auto-generated method stub
	}

	public double getPLD0() {
		return PLD0;
		// TODO Auto-generated method stub
	}

	public int getRadioReceiverSensitivity() {
		return RADIO_RECEIVER_SENSITIVITY;
		// TODO Auto-generated method stub
	}

	public double getRadioRxPowerConsumption() {
		return RADIO_RX_POWER_CONSUMPTION;
		// TODO Auto-generated method stub
	}

	public double[] getRadioTxPowerConsumptions() {
		return RADIO_TX_POWER_CONSUMPTIONS;
		// TODO Auto-generated method stub
	}

	public double[] getRadioTxPowerLevels() {
		return RADIO_TX_POWER_LEVELS;
		// TODO Auto-generated method stub
	}

	public abstract String getRadioName();
}
