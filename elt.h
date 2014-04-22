#ifndef ELT_H
#define ELT_H


#define AFSK_TX_POWER	7
// FCC ID, Lat, Long, UTC Fix, # Sat's, HDOP, Altitude, LiPoly, System In, AtMega
const uint8_t fccId[] = "KE7ZLH";

#define BEACON_NOTES 6
// static const uint16_t beaconNotes[BEACON_NOTES][3] = {{1067,704,7},{833,222,4},{684,264,3},{782,235,2},{605,296,1},{498,352,0}}; // Period, Iterations, TxPwr
static const uint16_t beaconNotes[BEACON_NOTES][3] = {{142,704,7},{113,222,4},{95,264,3},{106,235,2},{84,296,1},{71,352,0}}; // Period, Iterations, TxPwr
//	Note	A4		C#5 	E5 		D5 		F#5 	A5
//	Freq	440		554.4	659.3	587.3	740		880
//	uS		2273	1804	1517	1703	1351    1136
//	Halve these values to actually get the note, as the for loop times the half-wave gaps, not peak-peak wave shape
//	uS/2	1136	902		758		851		675		568
//	Times	0.8		0.2		0.2		0.2		0.2		0.2
//	TxPwr	7		4		3		2		1		0
//	In dBm	+20		+11		+8		+5		+2		+1
//	In mW	100		12.6	6.3		3.2		1.6		1.3
//	Actual	438.7	552.4	656.0	584.9	735.5	874.1 // With -66 uS already asserted
//	uS/2	1067	833		684		782		605		498 // 684 and 498 are slightly sharp and flat, respective
//	cycles	704		222		264		235		296		352 // Keep Cycle counts tied to actual periods, not corrected ones




void audioBlip(uint8_t, uint8_t, uint8_t);
void transmitELT(void);
void transmitELT_Beacon(void);
void transmitELT_Packet(void); //uint8_t *,uint8_t);
void transmitELT_AFSK(void);












#endif // ELT_H