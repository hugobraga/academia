package radios;


public final class RadioParameters {
	public static double RADIO_TX_POWER_LEVELS[];
	public static double RADIO_TX_POWER_CONSUMPTIONS[];
	public static double RADIO_RX_POWER_CONSUMPTION;
	
	public static int radioReceiverSensitivity;
	public static double PLD0;
	public static double MAXIMUM_DISTANCE_REACHED;
	
	public static double elecTx;
	public static double elecRx;
	public static double epsilon;
	
	public static String radioName;
	
	public final static int npathLossExp = 3;
	public final static double voltage = 3;
	
	public RadioParameters() {
		
		//here, we choose the radio
		Radio radio = new CC2420();
		
		RadioParameters.RADIO_TX_POWER_LEVELS = radio.getRadioTxPowerLevels();
		RadioParameters.RADIO_TX_POWER_CONSUMPTIONS = radio.getRadioTxPowerConsumptions();
		RadioParameters.RADIO_RX_POWER_CONSUMPTION = radio.getRadioRxPowerConsumption();
		RadioParameters.radioReceiverSensitivity = radio.getRadioReceiverSensitivity();
		RadioParameters.PLD0 = radio.getPLD0();
		RadioParameters.MAXIMUM_DISTANCE_REACHED = radio.getMaximumDistanceReached();
		RadioParameters.elecTx = radio.getElecTx();
		RadioParameters.elecRx = radio.getElecRx();
		RadioParameters.epsilon = radio.getEpsilon();
		RadioParameters.radioName = radio.getRadioName();
	}

	//returns the minimum tx level greater then txLevel
	private static double getMinRadioTxLevel(double txLevel) {
		for (int i = 0; i < RADIO_TX_POWER_LEVELS.length; i++) {
			if (RADIO_TX_POWER_LEVELS[i] >= txLevel) {
				//System.out.println("txLevel passado: "+txLevel+", RADIO_TX_POWER_LEVELS[i]"+RADIO_TX_POWER_LEVELS[i]);
				return RADIO_TX_POWER_LEVELS[i];
			}
		}
		return -1;
	}

	private static double getMinRadioTxConsumption(double txLevel) {
		for (int i = 0; i < RADIO_TX_POWER_LEVELS.length; i++) {
			if (RADIO_TX_POWER_LEVELS[i] >= txLevel)
				return RADIO_TX_POWER_CONSUMPTIONS[i];
		}
		return -1;
	}

	public static double getTxPowerConsumption(double distance) {
		//int requiredTx = (int)Math.ceil(radioReceiverSensitivity + PLD0 + 10*RadioParameters.npathLossExp*Math.log10(distance));
		double requiredTx = radioReceiverSensitivity + PLD0 + 10*RadioParameters.npathLossExp*Math.log10(distance);
		double minTxConsumption = getMinRadioTxConsumption(requiredTx);
		return minTxConsumption;    	
	}

	///*
	public static double getTxPower(double distance) {
		//double requiredTx = (int)Math.ceil(radioReceiverSensitivity + PLD0 + 10*RadioParameters.npathLossExp*Math.log10(distance));
		double requiredTx = radioReceiverSensitivity + PLD0 + 10*RadioParameters.npathLossExp*Math.log10(distance);
		double minTxLevel = getMinRadioTxLevel(requiredTx);
		return minTxLevel;
	}
	//*/

	public static double getMaxTxPowerConsumption() {
		return RADIO_TX_POWER_CONSUMPTIONS[RADIO_TX_POWER_CONSUMPTIONS.length-1];
	}

	public static double getTxConsumptionRatio(double distance) {
		return getTxPowerConsumption(distance)/getMaxTxPowerConsumption();
	}

	public static double getDistance(double txLevel) {
		//System.out.println("esta retornando a distancia para o txLevel: "+txLevel);
		return Math.pow(10, ((-radioReceiverSensitivity - PLD0+txLevel)/(10.0*RadioParameters.npathLossExp)));
	}

	///*
	public static double getRealDistance(double distance) {
		return RadioParameters.getDistance(getTxPower(distance));
	}
	//*/
}
