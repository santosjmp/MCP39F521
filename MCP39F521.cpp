/*
  MCP39F521.h - Library to be used with the MCP39F521,
  single-phase power-monitoring IC designed for
  real-time measurement of input power for AC/DC power
  supplies, providing power and energy values.
  Created by Jorge Santos, February 2, 2018.
*/

#include "Arduino.h"
#include "MCP39F521.h"

MCP39F521::MCP39F521()
{
	//Constructor
}

bool MCP39F521::begin(byte devAddress, TwoWire &wirePort)
{
  _devAddress = devAddress;
  _i2cPort = &wirePort;
  _i2cPort->begin();

}

/*Funções de leitura de output*/
int MCP39F521::read_SystemStatus() {
	byte result[2];
	unsigned int raw_status;

	if (read_register(_devAddress, SYSTEM_STATUS, 2, result)) {
		raw_status = result[1];
		raw_status = raw_status << 8;
		raw_status += result[0];

		return raw_status;
	}
	else
	{
		return 0.0;
	}
}

String MCP39F521::read_SystemVersion() {
	char dia = 0, mes = 0, ano = 0;
	byte result[2];


	if (read_register(_devAddress, SYSTEM_VERSION, 2, result)) {
		mes = (result[1] & 0x0F) - 1;
		ano = result[1] >> 4;
		dia = result[0];
		return String(String(dia, DEC) + "/" + String(mes, DEC) + "/" + String(ano + 2000, DEC));
	}
	else
	{
		return "Error";
	}


}

float MCP39F521::read_VRMS() {
	byte result[2];
	unsigned int raw_voltage;

	if (read_register(_devAddress, VOLTAGE_RMS, 2, result)) {
		raw_voltage = result[1];
		raw_voltage = raw_voltage << 8;
		raw_voltage += result[0];

		return raw_voltage / 10.0;
	}
	else
	{
		return 0.0;
	}
}

float MCP39F521::read_IRMS() {
	byte result[4];
	long raw_current;

	if (read_register(_devAddress, CURRENT_RMS, 4, result)) {
		raw_current = result[3];
		raw_current = raw_current << 8;
		raw_current += result[2];
		raw_current = raw_current << 8;
		raw_current += result[1];
		raw_current = raw_current << 8;
		raw_current += result[0];

		return raw_current/10000.0 ;
	}
	else
	{
		return 0.0;
	}
}

float MCP39F521::read_PowerFactor() {
	byte result[2];
	int raw_PF;

	if (read_register(_devAddress, POWER_FACTOR, 2, result)) {
		raw_PF = result[1];
		raw_PF = raw_PF << 8;
		raw_PF += result[0];
		return raw_PF/32768.0;
	}
	else
	{
		return 0.0;
	}
}

float MCP39F521::read_Freq() {
	byte result[2];
	unsigned int raw_frequency;

	if (read_register(_devAddress, LINE_FREQUENCY, 2, result)) {
		raw_frequency = result[1];
		raw_frequency = raw_frequency << 8;
		raw_frequency += result[0];

		return raw_frequency / 1000.0;
	}
	else
	{
		return 0.0;
	}
}

float MCP39F521::read_Active() {
	byte result[4];
	long raw_active;

	if (read_register(_devAddress, ACTIVE_POWER, 4, result)) {
		raw_active = result[3];
		raw_active = raw_active << 8;
		raw_active += result[2];
		raw_active = raw_active << 8;
		raw_active += result[1];
		raw_active = raw_active << 8;
		raw_active += result[0];

		return raw_active / 100.0;
	}
	else
	{
		return 0.0;
	}
}

float MCP39F521::read_Reactive() {
	byte result[4];
	long raw_reactive;

	if (read_register(_devAddress, REACTIVE_POWER, 4, result)) {
		raw_reactive = result[3];
		raw_reactive = raw_reactive << 8;
		raw_reactive += result[2];
		raw_reactive = raw_reactive << 8;
		raw_reactive += result[1];
		raw_reactive = raw_reactive << 8;
		raw_reactive += result[0];

		return raw_reactive / 100.0;
	}
	else
	{
		return 0.0;
	}
}

float MCP39F521::read_Apparent() {
	byte result[4];
	long raw_apparent;

	if (read_register(_devAddress, APPARENT_POWER, 4, result)) {
		raw_apparent = result[3];
		raw_apparent = raw_apparent << 8;
		raw_apparent += result[2];
		raw_apparent = raw_apparent << 8;
		raw_apparent += result[1];
		raw_apparent = raw_apparent << 8;
		raw_apparent += result[0];

		return raw_apparent/100.0;
	}
	else
	{
		return 0.0;
	}
}

long long MCP39F521::read_Imp_Act_Ener() {
	byte result[8];
	long long impActEnergy;

	if (read_register(_devAddress, IMPORT_ACTIVE_ENERGY_COUNTER, 8, result)) {
		impActEnergy = result[7];
		impActEnergy = impActEnergy << 8;
		impActEnergy += result[6];
		impActEnergy = impActEnergy << 8;
		impActEnergy += result[5];
		impActEnergy = impActEnergy << 8;
		impActEnergy += result[4];
		impActEnergy = impActEnergy << 8;
		impActEnergy += result[3];
		impActEnergy = impActEnergy << 8;
		impActEnergy += result[2];
		impActEnergy = impActEnergy << 8;
		impActEnergy += result[1];
		impActEnergy = impActEnergy << 8;
		impActEnergy += result[0];
	}
	return impActEnergy;
}

long long MCP39F521::read_Exp_Act_Ener() {
	byte result[8];
	long long expActEnergy;

	if (read_register(_devAddress, EXPORT_ACTIVE_ENERGY_COUNTER, 8, result)) {
		expActEnergy = result[7];
		expActEnergy = expActEnergy << 8;
		expActEnergy += result[6];
		expActEnergy = expActEnergy << 8;
		expActEnergy += result[5];
		expActEnergy = expActEnergy << 8;
		expActEnergy += result[4];
		expActEnergy = expActEnergy << 8;
		expActEnergy += result[3];
		expActEnergy = expActEnergy << 8;
		expActEnergy += result[2];
		expActEnergy = expActEnergy << 8;
		expActEnergy += result[1];
		expActEnergy = expActEnergy << 8;
		expActEnergy += result[0];
	}
	return expActEnergy;
}

long long MCP39F521::read_Imp_React_Ener() {
	byte result[8];
	long long impReactEnergy;

	if (read_register(_devAddress, IMPORT_REACTIVE_ENERGY_COUNTER, 8, result)) {
		impReactEnergy = result[7];
		impReactEnergy = impReactEnergy << 8;
		impReactEnergy += result[6];
		impReactEnergy = impReactEnergy << 8;
		impReactEnergy += result[5];
		impReactEnergy = impReactEnergy << 8;
		impReactEnergy += result[4];
		impReactEnergy = impReactEnergy << 8;
		impReactEnergy += result[3];
		impReactEnergy = impReactEnergy << 8;
		impReactEnergy += result[2];
		impReactEnergy = impReactEnergy << 8;
		impReactEnergy += result[1];
		impReactEnergy = impReactEnergy << 8;
		impReactEnergy += result[0];
	}
	return impReactEnergy;
}

long long MCP39F521::read_Exp_React_Ener() {
	byte result[8];
	long long expReactEnergy;

	if (read_register(_devAddress, EXPORT_REACTIVE_ENERGY_COUNTER, 8, result)) {
		expReactEnergy = result[7];
		expReactEnergy = expReactEnergy << 8;
		expReactEnergy += result[6];
		expReactEnergy = expReactEnergy << 8;
		expReactEnergy += result[5];
		expReactEnergy = expReactEnergy << 8;
		expReactEnergy += result[4];
		expReactEnergy = expReactEnergy << 8;
		expReactEnergy += result[3];
		expReactEnergy = expReactEnergy << 8;
		expReactEnergy += result[2];
		expReactEnergy = expReactEnergy << 8;
		expReactEnergy += result[1];
		expReactEnergy = expReactEnergy << 8;
		expReactEnergy += result[0];
	}
	return expReactEnergy;
}

/*Funções de configuração do MCP39F521*/
void MCP39F521::set_PGAGain( byte channel, byte gain) {
	byte config_gain, msb_byte, new_msb_byte = 0;
	long current_config, new_config, config_msb;
	char temp_bit = 0;

	current_config = read_Configuration(_devAddress);
	msb_byte = (current_config >> 24);
	config_gain = log2(gain);

	switch (channel)
	{
	case 0:
		//Serial.println("canal 0");
		for (int i = 0; i < 8; i++) {
			if (i < 3) {
				bitWrite(new_msb_byte, i, bitRead(config_gain, i));
			}
			else
			{
				bitWrite(new_msb_byte, i, bitRead(msb_byte, i));
			}
		}
		break;
	case 1:
		//Serial.println("canal 1");
		for (int i = 0; i < 8; i++) {
			if ((i > 2) && (i < 6)) {
				bitWrite(new_msb_byte, i, bitRead(config_gain, i - 3));
			}
			else
			{
				bitWrite(new_msb_byte, i, bitRead(msb_byte, i));
			}
		}
		break;
	default:
		break;
	}
	new_config = current_config & 0x00FFFFFF;
	config_msb = new_msb_byte << 24;
	new_config += config_msb;
	write_Configuration(_devAddress, new_config);
}

byte MCP39F521::read_PGAGain( byte channel) {
	byte result, gain;
	result = read_Configuration(_devAddress) >> 24;

	switch (channel)
	{
	case 0:
		gain = result & 0x07;
		break;
	case 1:
		gain = (result & 0x38) >> 3;
		break;
	default:
		break;
	}
	return pow(2, gain);
}

int MCP39F521::read_Gain_VRMS() {
	byte result[2];
	unsigned int raw_gainV;

	if (read_register(_devAddress, GAIN_VOLTAGE_RMS, 2, result)) {
		raw_gainV = result[1];
		raw_gainV = raw_gainV << 8;
		raw_gainV += result[0];

		return raw_gainV;
	}
	else
	{
		return 0;
	}
}

int MCP39F521::read_Gain_IRMS() {
	byte result[2];
	unsigned int raw_gainI;

	if (read_register(_devAddress, GAIN_CURRENT_RMS, 2, result)) {
		raw_gainI = result[1];
		raw_gainI = raw_gainI << 8;
		raw_gainI += result[0];

		return raw_gainI;
	}
	else
	{
		return 0;
	}
}

int MCP39F521::read_Gain_ActiveP() {
	byte result[2];
	unsigned int raw_gainP;

	if (read_register(_devAddress, GAIN_ACTIVE_POWER, 2, result)) {
		raw_gainP = result[1];
		raw_gainP = raw_gainP << 8;
		raw_gainP += result[0];

		return raw_gainP;
	}
	else
	{
		return 0;
	}
}

int MCP39F521::read_Gain_ReactiveP() {
	byte result[2];
	unsigned int raw_gainQ;

	if (read_register(_devAddress, GAIN_REACTIVE_POWER, 2, result)) {
		raw_gainQ = result[1];
		raw_gainQ = raw_gainQ << 8;
		raw_gainQ += result[0];

		return raw_gainQ;
	}
	else
	{
		return 0;
	}
}

void MCP39F521::set_Gain_VRMS( unsigned int volt_calib) {
	byte result, data[2];

	data[0] = lowByte(volt_calib);
	data[1] = highByte(volt_calib);

	write_register(_devAddress, GAIN_VOLTAGE_RMS, 2, data);
}

void MCP39F521::set_Gain_IRMS( unsigned int amp_calib) {
	byte result, data[2];

	data[0] = lowByte(amp_calib);
	data[1] = highByte(amp_calib);

	write_register(_devAddress, GAIN_CURRENT_RMS, 2, data);
}

void MCP39F521::set_Gain_ActiveP( unsigned int actP_calib) {
	byte result, data[2];

	data[0] = lowByte(actP_calib);
	data[1] = highByte(actP_calib);

	write_register(_devAddress, GAIN_ACTIVE_POWER, 2, data);
}

void MCP39F521::set_Gain_ReactiveP( unsigned int reactP_calib) {
	byte result, data[2];

	data[0] = lowByte(reactP_calib);
	data[1] = highByte(reactP_calib);

	write_register(_devAddress, GAIN_REACTIVE_POWER, 2, data);
}

long MCP39F521::read_OffsetCurrent() {
	byte result[4];
	long raw_offsetI;

	if (read_register(_devAddress, OFFSET_CURRENT_RMS, 4, result)) {
		raw_offsetI = result[3];
		raw_offsetI = raw_offsetI << 8;
		raw_offsetI += result[2];
		raw_offsetI = raw_offsetI << 8;
		raw_offsetI += result[1];
		raw_offsetI = raw_offsetI << 8;
		raw_offsetI += result[0];

		return raw_offsetI;
	}
	else
	{
		return 0;
	}
}

long MCP39F521::read_OffsetActive() {
	byte result[4];
	long raw_offsetP;

	if (read_register(_devAddress, OFFSET_ACTIVE_POWER, 4, result)) {
		raw_offsetP = result[3];
		raw_offsetP = raw_offsetP << 8;
		raw_offsetP += result[2];
		raw_offsetP = raw_offsetP << 8;
		raw_offsetP += result[1];
		raw_offsetP = raw_offsetP << 8;
		raw_offsetP += result[0];

		return raw_offsetP;
	}
	else
	{
		return 0;
	}
}

long MCP39F521::read_OffsetReactive() {
	byte result[4];
	long raw_offsetQ;

	if (read_register(_devAddress, OFFSET_REACTIVE_POWER, 4, result)) {
		raw_offsetQ = result[3];
		raw_offsetQ = raw_offsetQ << 8;
		raw_offsetQ += result[2];
		raw_offsetQ = raw_offsetQ << 8;
		raw_offsetQ += result[1];
		raw_offsetQ = raw_offsetQ << 8;
		raw_offsetQ += result[0];

		return raw_offsetQ;
	}
	else
	{
		return 0;
	}
}

void MCP39F521::set_OffsetCurrent( long offset_I) {
	byte result, data[4];

	data[0] = offset_I;
	data[1] = (offset_I>>8);
	data[2] = (offset_I >> 16);
	data[3] = (offset_I >> 24);


	write_register(_devAddress, OFFSET_CURRENT_RMS, 4, data);
}

void MCP39F521::set_OffsetActive( long offset_P) {
	byte result, data[4];

	data[0] = offset_P;
	data[1] = (offset_P >> 8);
	data[2] = (offset_P >> 16);
	data[3] = (offset_P >> 24);


	write_register(_devAddress, OFFSET_ACTIVE_POWER, 4, data);
}

void MCP39F521::set_OffsetReactive( long offset_Q) {
	byte result, data[4];

	data[0] = offset_Q;
	data[1] = (offset_Q >> 8);
	data[2] = (offset_Q >> 16);
	data[3] = (offset_Q >> 24);


	write_register(_devAddress, OFFSET_REACTIVE_POWER, 4, data);
}

boolean MCP39F521::read_Range( byte *r_power, byte *r_current, byte *r_voltage) {
	byte result[4];

	if (read_register(_devAddress, RANGE, 4, result)) {
		*r_power = result[2];
		*r_current = result[1];
		*r_voltage = result[0];

		return true;
	}
	else
	{
		return false;
	}
}

boolean MCP39F521::set_V_Range( byte r_voltage) {
	byte result, rV[] = { r_voltage };
	//Serial.println(rV[0]);
	result = write_register(_devAddress, (RANGE), 1, rV);
	//Serial.println(result);
	if (result == 0) {
		return true;
	}
	else {
		return false;
	}
}

boolean MCP39F521::set_I_Range( byte r_current) {
	byte result, rI[] = { r_current };
	//Serial.println(rI[0]);
	result = write_register(_devAddress, (RANGE + 1), 1, rI);
	//Serial.println(result);
	if (result == 0) {
		return true;
	}
	else {
		return false;
	}
}

boolean MCP39F521::set_P_Range( byte r_power) {
	byte result, rP[] = { r_power };
	//Serial.println(rP[0]);
	result = write_register(_devAddress, (RANGE + 2), 1, rP);
	//Serial.println(result);
	if (result == 0) {
		return true;
	}
	else {
		return false;
	}
}

int MCP39F521::read_Phase_Comp() {
	byte result[2];
	unsigned int raw_Phase;

	if (read_register(_devAddress, PHASE_COMPENSATION, 2, result)) {
		raw_Phase = result[1];
		raw_Phase = raw_Phase << 8;
		raw_Phase += result[0];

		return raw_Phase;
	}
	else
	{
		return 0;
	}
}

void MCP39F521::set_Phase_Comp( int phase_comp) {
	byte result, data[2];

	data[0] = lowByte(phase_comp);
	data[1] = highByte(phase_comp);

	write_register(_devAddress, PHASE_COMPENSATION, 2, data);
}

int MCP39F521::read_ApparentPowerDivisor() {
	byte result[2];
	unsigned int raw_APD;

	if (read_register(_devAddress, APPARENT_POWER_DIVISOR, 2, result)) {
		raw_APD = result[1];
		raw_APD = raw_APD << 8;
		raw_APD += result[0];

		return raw_APD;
	}
	else
	{
		return 0;
	}
}

int MCP39F521::read_EnergyAcum_Control() {
	byte result[2];
	unsigned int raw_EAControl;

	if (read_register(_devAddress, ENERGY_CONTROL, 2, result)) {
		raw_EAControl = result[1];
		raw_EAControl = raw_EAControl << 8;
		raw_EAControl += result[0];

		return raw_EAControl;
	}
	else
	{
		return 0.0;
	}
}

void MCP39F521::set_EnergyAcum_Control( byte EA_state) {
	write_register(_devAddress, ENERGY_CONTROL, 1, &EA_state);
}

void MCP39F521::save_registers_flash() {
	byte checksum, result;
	byte dados[3] = { HEADER,0x04,SAVE_REGISTERS_FLASH };

	checksum = calc_checksum(dados, 3);

	_i2cPort->beginTransmission(_devAddress);
	_i2cPort->write(HEADER);
	_i2cPort->write(0x04);
	_i2cPort->write(SAVE_REGISTERS_FLASH);
	_i2cPort->write(checksum);
	result = _i2cPort->endTransmission();
}


/*Implementação dos comandos principais do MCP39F521*/
boolean MCP39F521::read_register(byte devAddress, unsigned int register_to_read, byte n_bytes_read, byte data[]) {

	const byte bytes_frame = 0x08;
	byte result, header, bytes_rx, sended_CRC, received_CRC, calc_CRC, temp_data[MAX_BYTE_READ];

	byte payload[] = { HEADER,bytes_frame,SET_ADDRESS_POINTER,highByte(register_to_read), lowByte(register_to_read),REGISTER_READ,n_bytes_read };

	sended_CRC = calc_checksum(payload, 7);

	_i2cPort->beginTransmission(devAddress);
	_i2cPort->write(HEADER);
	_i2cPort->write(bytes_frame);
	set_pointer(register_to_read);
	_i2cPort->write(REGISTER_READ);
	_i2cPort->write(n_bytes_read);
	_i2cPort->write(sended_CRC);
	result = _i2cPort->endTransmission();

	_i2cPort->requestFrom((uint)devAddress, n_bytes_read + 3);
	for (int i = 0; i < n_bytes_read + 3; i++) {
		temp_data[i] = _i2cPort->read();
	}

	calc_CRC = calc_checksum(temp_data, n_bytes_read + 2);
	if (calc_CRC == temp_data[n_bytes_read + 2]) {
		for (int i = 0; i < temp_data[1] - 3; i++) {
			data[i] = temp_data[i + 2];
		}
		return true;
	}
	else {
		return false;
	}
}

void MCP39F521::set_pointer(unsigned int address) {

	byte address_low = lowByte(address), address_high = highByte(address);

	_i2cPort->write(SET_ADDRESS_POINTER);
	_i2cPort->write(address_high);
	_i2cPort->write(address_low);

}

byte MCP39F521::write_register(byte devAddress, unsigned int register_to_write, unsigned int bytes_to_write, byte data[]) {
	byte checksum;
	byte bytes_frame = 0x08 + bytes_to_write;

	byte payload[MAX_BYTE_WRITE];

	payload[0] = HEADER;
	payload[1] = bytes_frame;
	payload[2] = SET_ADDRESS_POINTER;
	payload[3] = highByte(register_to_write);
	payload[4] = lowByte(register_to_write);
	payload[5] = REGISTER_WRITE;
	payload[6] = bytes_to_write;
	for (int i = 7; i < bytes_to_write + 7; i++) {
		payload[i] = data[i - 7];
	}
	checksum = calc_checksum(payload, bytes_frame - 1);

	_i2cPort->beginTransmission(devAddress);
	_i2cPort->write(HEADER);
	_i2cPort->write(bytes_frame);
	set_pointer(register_to_write);
	_i2cPort->write(REGISTER_WRITE);
	_i2cPort->write(bytes_to_write);
	for (int i = 0; i < bytes_to_write; i++) {
		//Serial.println(data[i]);
		_i2cPort->write(data[i]);
	}
	_i2cPort->write(checksum);
	delay(200);
	return _i2cPort->endTransmission();
}

/*funções de apoio*/
byte MCP39F521::calc_checksum(byte terms[], int n_terms) {
	int sum = 0;
	byte remainder = 0;

	for (int i = 0; i < n_terms; i++) {
		sum += terms[i];
	}
	remainder = sum % 256;

	return remainder;

}

void MCP39F521::write_Configuration(byte devAddress, long config) {
	byte result, data[4];

	data[0] = lowByte(config);
	config = config >> 8;
	data[1] = lowByte(config);
	config = config >> 8;
	data[2] = lowByte(config);
	config = config >> 8;
	data[3] = lowByte(config);

	write_register(devAddress, SYSTEM_CONFIGURATION, 4, data);
}

long MCP39F521::read_Configuration(byte devAddress) {
	byte result[4];
	long raw_config;

	if (read_register(devAddress, SYSTEM_CONFIGURATION, 4, result)) {
		raw_config = result[3];
		raw_config = raw_config << 8;
		raw_config += result[2];
		raw_config = raw_config << 8;
		raw_config += result[1];
		raw_config = raw_config << 8;
		raw_config += result[0];

		return raw_config;
	}
	else
	{
		return 0;
	}
}

/*void MCP39F521::on_event_call( void(*function)(int), int pin) {		//Funçao para onde guarda o ponteiro da funçao para chamar mais tarde 

	pinMode(pin, INPUT);
	attachInterrupt(digitalPinToInterrupt(pin), evento, RISING);

	_function_event; = function;

}
*/
/*unsigned int MCP39F521::ler_status_evento(){
	byte result[2];
	unsigned int stat;

	read_register(_devAddress, SYSTEM_STATUS, 2, result);

	stat = result[1];
	stat = stat << 8;
	stat += result[0];

	return stat;
}*/

unsigned int MCP39F521::ler_registo_evento() {
	byte result[4];
	unsigned int reg;

	if (read_register(_devAddress, EVENT_CONFIGURATION, 4, result)) {

		reg = result[3];
		reg = reg << 8;
		reg += result[2];
		reg = reg << 8;
		reg += result[1];
		reg = reg << 8;
		reg += result[0];

		return reg;
	}
	else {
		return NULL;
	}
}


int MCP39F521::evento() {
	int resp;
	int stat;

	stat = read_SystemStatus();

	if (bitRead(stat, 0)) {
		resp = VOLTAGE_SAG;
	}
	else if (bitRead(stat, 1)) {
		resp = VOLTAGE_SURGE;
	}
	else if (bitRead(stat, 2)) {
		resp = OVER_CURRENT;
	}
	else if (bitRead(stat, 3)) {
		resp = OVER_POWER;
	}
	else
	{
		resp = NULL;
	}

	return resp;
	//_function_event(resp);
}

void MCP39F521::set_event_over_pow_limit(unsigned int limit) {
	byte data[2];

	data[0] = limit;
	data[1] = (limit >> 8);

	write_register(_devAddress, OVER_POWER_LIMIT, 2, data);
}

void MCP39F521::set_event_over_curr_limit(unsigned int limit) {
	byte data[2];

	data[0] = limit;
	data[1] = (limit >> 8);

	write_register(_devAddress, OVER_CURRENT_LIMIT, 2, data);
}

void MCP39F521::set_event_vsurge_limit(unsigned int limit) {
	byte data[2];

	data[0] = limit;
	data[1] = (limit >> 8);

	write_register(_devAddress, VOLTAGE_SURGE_LIMIT, 2, data);
}

void MCP39F521::set_event_vsag_limit(unsigned int limit) {
	byte data[2];

	data[0] = limit;
	data[1] = (limit >> 8);

	write_register(_devAddress, VOLTAGE_SAG_LIMIT, 2, data);
}

void MCP39F521::set_event_over_pow(unsigned int config) {

	unsigned int stat;
	stat = ler_registo_evento();

	if (stat) {
		if (config == ENABLE) {
			bitSet(stat, 19);
		}
		if (config == DESABLE) {
			bitClear(stat, 19);
		}
		if (config == CLEAR_ON) {
			bitSet(stat, 10);
		}
		if (config == CLEAR_OFF) {
			bitClear(stat, 10);
		}
		if (config == LATCH_ON) {
			bitSet(stat, 6);
		}
		if (config == LATCH_OFF) {
			bitClear(stat, 6);
		}
		if (config == TEST_ON) {
			bitSet(stat, 1);
		}
		if (config == TEST_OFF) {
			bitClear(stat, 1);
		}

		byte  data[4];

		data[0] = stat;
		data[1] = (stat >> 8);
		data[2] = (stat >> 16);
		data[3] = (stat >> 24);

		write_register(_devAddress, EVENT_CONFIGURATION, 4, data);
	}


}

void MCP39F521::set_event_over_curr(unsigned int config) {

	unsigned int stat;


	stat = ler_registo_evento();

	if (stat) {
		if (config == ENABLE) {
			bitSet(stat, 18);
		}
		if (config == DESABLE) {
			bitClear(stat, 18);
		}
		if (config == CLEAR_ON) {
			bitSet(stat, 11);
		}
		if (config == CLEAR_OFF) {
			bitClear(stat, 11);
		}
		if (config == LATCH_ON) {
			bitSet(stat, 4);
		}
		if (config == LATCH_OFF) {
			bitClear(stat, 4);
		}
		if (config == TEST_ON) {
			bitSet(stat, 0);
		}
		if (config == TEST_OFF) {
			bitClear(stat, 0);
		}

		byte  data[4];

		data[0] = stat;
		data[1] = (stat >> 8);
		data[2] = (stat >> 16);
		data[3] = (stat >> 24);

		write_register(_devAddress, EVENT_CONFIGURATION, 4, data);
	}
}

void MCP39F521::event_vsurge(unsigned int config) {

	unsigned int stat;


	stat = ler_registo_evento();

	if (stat) {
		if (config == ENABLE) {
			bitSet(stat, 17);
		}
		if (config == DESABLE) {
			bitClear(stat, 17);
		}
		if (config == CLEAR_ON) {
			bitSet(stat, 9);
		}
		if (config == CLEAR_OFF) {
			bitClear(stat, 9);
		}
		if (config == LATCH_ON) {
			bitSet(stat, 7);
		}
		if (config == LATCH_OFF) {
			bitClear(stat, 7);
		}
		if (config == TEST_ON) {
			bitSet(stat, 3);
		}
		if (config == TEST_OFF) {
			bitClear(stat, 3);
		}

		byte  data[4];

		data[0] = stat;
		data[1] = (stat >> 8);
		data[2] = (stat >> 16);
		data[3] = (stat >> 24);

		write_register(_devAddress, EVENT_CONFIGURATION, 4, data);
	}
}

void MCP39F521::event_vsag(unsigned int config) {

	unsigned int stat;


	stat = ler_registo_evento();

	if (stat) {
		if (config == ENABLE) {
			bitSet(stat, 16);
		}
		if (config == DESABLE) {
			bitClear(stat, 16);
		}
		if (config == CLEAR_ON) {
			bitSet(stat, 8);
		}
		if (config == CLEAR_OFF) {
			bitClear(stat, 8);
		}
		if (config == LATCH_ON) {
			bitSet(stat, 6);
		}
		if (config == LATCH_OFF) {
			bitClear(stat, 6);
		}
		if (config == TEST_ON) {
			bitSet(stat, 2);
		}
		if (config == TEST_OFF) {
			bitClear(stat, 2);
		}

		byte  data[4];

		data[0] = stat;
		data[1] = (stat >> 8);
		data[2] = (stat >> 16);
		data[3] = (stat >> 24);

		write_register(_devAddress, EVENT_CONFIGURATION, 4, data);
	}
}

void MCP39F521::set_event_manu(unsigned int config) {

	unsigned int stat;


	stat = ler_registo_evento();

	if (stat) {
		if (config == ENABLE) {
			bitSet(stat, 14);

		}
		if (config == DESABLE) {
			bitClear(stat, 14);
		}
		Serial.println(stat, BIN);

		byte  data[4];

		data[0] = stat;
		data[1] = (stat >> 8);
		data[2] = (stat >> 16);
		data[3] = (stat >> 24);

		write_register(_devAddress, EVENT_CONFIGURATION, 4, data);
	}
}

void MCP39F521::set_event_config_all(unsigned int registo) {
	byte  data[4];

	data[0] = registo;
	data[1] = (registo >> 8);
	data[2] = (registo >> 16);
	data[3] = (registo >> 24);

	write_register(_devAddress, EVENT_CONFIGURATION, 4, data);
}
