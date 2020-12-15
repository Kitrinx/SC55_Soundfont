// SC55 ROM to Soundfont converter
// Code by Kitrinx and NewRisingSun

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#ifdef __GNUC__
	#define packed_struct struct __attribute__((__packed__))
	#define PATH_DIV "/"
#else
	#define packed_struct _Pragma("pack(1)") struct
	#define PATH_DIV "\\"
#endif

#include "riff.h"
#include "sf2.h"

enum partial_bytes {
	pp_panpot             = 5,
	pp_course_pitch       = 6, // Shifts the instrument key
	pp_fine_pitch         = 7,
	pp_random_pitch       = 8,
	pp_note_range         = 9,
	pp_vibrato_depth      = 10,
	pp_part_attenuation   = 65,
	pp_lfo_depth          = 68,
	pp_tva_p1_vol         = 70,
	pp_tva_p2_vol         = 71,
	pp_tva_p3_vol         = 72,
	pp_tva_p4_vol         = 73,
	pp_tva_p1_len         = 74,
	pp_tva_p2_len         = 75,
	pp_tva_p3_len         = 76,
	pp_tva_p4_len         = 77,
	pp_tva_p5_len         = 78,
};

enum inst_header_bytes {
	ih_attenuation        = 0,
	ih_note_flags         = 1,
	ih_reverb             = 3,
	ih_chorus             = 4,
	ih_panpot             = 5,
	ih_partial_en         = 6,
};

#define BLOCK2 0x2A9A1

uint32_t banks_vsc[8] = {0x00034, 0x0BD34, 0x0DEF4, 0x10034, 0x1BD34, 0x1DEF4, 0x20034, 0x30000};
uint32_t banks_sc55[8] ={0x10000, 0x1BD00, 0x1DEC0, 0x20000, 0x2BD00, 0x2DEC0, 0x30000, 0x38080};
#define B1_SZ 0xD8
#define B2_SZ 0x3C
#define B3_SZ 0x10
#define B4_SZ 0x100
#define NAME_SZ 0xC
#define NUM_INST (224 * 2)
#define NUM_PARTS (144 * 2)
#define NUM_SAMPLES (532 * 2)
#define NUM_VARIATIONS 128
#define NUM_DRUMS 14
#define KIT_SIZE 0x48C
#define INT24_MAX 0x7FFFFF
#define INT24_MIN (0x7FFFFF * -1)
#define BOOST_AMOUNT 0.0

#define MKII

#if defined(MKII)
	#define CTRL_VER_ADDR 0x3D148
#else
	#define CTRL_VER_ADDR 0xF380
#endif

#define MAG2DB(x) (-200.0 * log10(x))
#define SC552AMP(x) (0.1 * pow(2.0, (double)(x) / 36.7111) - 0.1)
#define PCT2VOL(x) ((x) == 127 ? 0 : ((x) > 0 ? MAG2DB(SC552AMP(x)) : 1440))

// 0x3CA48 - 0x70 len settings, 0x700 total

int32_t compress_sample(long double sample, long double value)
{
	long double comp_f2 = value;
	long double comp_a2 = value / 2.0;
	long double comp_x2 = (((double)INT24_MAX * (comp_f2 - 1.0)) / ((comp_f2 * comp_a2) - 1.0)) + 1.0; // +1 to make sure it won't overflow
	long double comp_b2 = comp_x2 * comp_a2;

	long double v  = (sample < 0) ? -1.0 * sample : sample;
	long double v2 = (v < comp_x2) ? (v * comp_a2) : (((v - comp_x2) / comp_f2) + comp_b2);
	long double compr =  (sample < 0) ? -1.0 * v2 : v2;

	return (int32_t) roundl(compr);
}

packed_struct ins_partial { // 92 bytes
	uint8_t spacer1;          // Always 0
	uint8_t unknown1;         // Has value, usually 0x40
	uint16_t part_index;      // Part table index, 0xFFFF for unused
	uint8_t pp[88];// Unknown set of Part parameters
};

packed_struct instrument { // 204 bytes
	char name[NAME_SZ];
	uint8_t header[20];   // {7F, 10, 00, 3C, 4F, 01, 02?}  // {FF?, FF?, FF, FF, FF, FF} // {00?, 00, 00, 00, 00, 00}
	struct ins_partial parts[2];
};

packed_struct part { // 48 bytes
	char name[NAME_SZ];
	uint8_t breaks[16];  // Note breakpoints corresponding to sample addresses
	uint16_t samples[16]; // Set of addresses to the sample table. 0 is default, and above corresponds to breakpoints
};

packed_struct sample { // 16 bytes
	uint8_t  volume;      // Volume attenuation 7F to 0
	uint8_t  offset[3];   // Offset on vsc, bank + scrambled address on SC55. Bits above 20 are wave bank.
	uint16_t attack_end;  // boundry between attack and decay? Unconfirmed.
	uint16_t sample_len;  // Sample Size
	uint16_t loop_len;    // Loop point, used as sample_len - loop_len - 1
	uint8_t  loop_mode;   // 2 if not a looping sound, 1 forward then back, 0 forward only.
	uint8_t  root_key;    // Base pitch of the sample
	uint16_t pitch;       // Fine pitch adjustment, 2048 to 0. Positive increases pitch.
	uint16_t fine_volume; // Always 0x400 on VSC, appears to be 1000ths of a decibel. Positive is higher volume.
};

packed_struct drum { //1164 bytes
	uint16_t preset[128];
	uint8_t volume[128];
	uint8_t key[128];
	uint8_t assignGroup[128];// AKA exclusive class
	uint8_t panpot[128];
	uint8_t reverb[128];
	uint8_t chorus[128];
	uint8_t flags[128];// 0x10 == responds to note on 0x01 responds to note_off
	char name[NAME_SZ];
};

packed_struct variation {
	uint16_t variation[128];
};

packed_struct synth {
	struct drum drums[NUM_DRUMS];
	struct variation variations[NUM_VARIATIONS];
	struct instrument instruments[NUM_INST];
	struct part parts[NUM_PARTS];
	struct sample samples[NUM_SAMPLES];
	uint8_t wave_data[0x300000];
	uint8_t control_data[0x40000];
};

struct sf_instruments {
	int32_t used_inst[NUM_INST * 10];
	struct sfInst inst[NUM_INST * 100];
	struct sfInstBag ibag[NUM_INST * 100 * 10];
	struct sfModList imod[NUM_INST * 100 * 10];
	struct sfInstGenList igen[NUM_INST * 100 * 10];
	uint32_t inst_count;
	uint32_t ibag_count;
	uint32_t imod_count;
	uint32_t igen_count;
};

struct pf_used_inst {
	bool used;
	uint32_t partial0;
	uint32_t partial1;
};

struct sf_presets {
	struct pf_used_inst used[NUM_INST * 10];
	struct sfPresetHeader phdr[NUM_INST * 2];
	struct sfPresetBag pbag[NUM_INST * 100];
	struct sfModList pmod[NUM_INST * 100];
	struct sfGenList pgen[NUM_INST * 100];
	uint32_t phdr_count;
	uint32_t pbag_count;
	uint32_t pmod_count;
	uint32_t pgen_count;
};

typedef struct sample_params {
	double t1; // Times in seconds
	double t2;
	double t3;
	double t4;
	double t5;
	double l1; // Levels in amplitude
	double l2;
	double l3;
	double l4;
	uint8_t s1; // Evelope Shapes. 0 = linear, 1 = Concave 2 = Convex
	uint8_t s2;
	uint8_t s3;
	uint8_t s4;
	uint8_t s5;
	uint32_t terminal_phase; // The phase that reaches the terminal level
} sample_params;

packed_struct sample_record {
	struct sample_params params;
	uint32_t sample_number;
	bool used;
};


#define MAX_SF_SAMPLES UINT16_MAX
struct sf_samples {
	struct sfSample shdr[MAX_SF_SAMPLES];
	int16_t *sample16;
	uint8_t *sample8;
	uint32_t num_samples;
	size_t data_size;
	struct sample_record used[NUM_SAMPLES][12];
};

packed_struct wav_params {
	uint16_t audio_format;
	uint16_t num_channels;
	uint32_t sample_rate;
	uint32_t byte_rate;
	uint16_t block_align;
	uint16_t bits_per_sample;
};

struct phase_data {
	float durationInSeconds;
	int endLevel;
};

typedef struct {
	int nn;
	int nd;
	long double n[3];
	long double d[3];
} biquad;

// Unknown tables in control rom:
// 0x3EF82 0xFA2 UINT16_T
// 0x3CA48 - 3D147 0x10 groups of 0x70 UINT8_T

#define TP1 (long double)(3180e-6)
#define TP2 (long double)(75e-6)
#define TZ1 (long double)(318e-6)
#define TZ2 (long double)(3.18e-6)

#define PZ(T) exp(-1.0/(fs*(T)))

biquad compute_riaa_irr(long double fs, long double dcgain, int extrazero)
{
	static biquad bq;
	long double p1, p2, z1, z2, gain;
	int i;

	p1 = PZ(TP1);
	p2 = PZ(TP2);
	z1 = PZ(TZ1);
	z2 = PZ(TZ2);

	bq.nd = 3;
	bq.d[0] = 1.0;
	bq.d[1] = -p1-p2;
	bq.d[2] = p1*p2;

	if(extrazero) {
		bq.nn = 3;
		bq.n[0] = 1.0;
		bq.n[1] = -z1-z2;
		bq.n[2] = z1*z2;
	} else {
		bq.nn = 2;
		bq.n[0] = 1.0;
		bq.n[1] = -z1;
		bq.n[2] = 0.0;
	}
	gain = (bq.n[0]+bq.n[1]+bq.n[2])/(1.0+bq.d[1]+bq.d[2]);
	long double gain_atten = (dcgain/gain);
	for(i=0; i<3; i++) {
		bq.n[i] *= gain_atten;
	}

	return bq;
}

void iir_filter(const long double *b, const long double *a, size_t filterLength, const long double *in, long double *out, size_t length)
{
	const long double a0 = a[0];
	const long double *a_end = &a[filterLength-1];
	const long double *out_start = out;
	a++;
	out--;
	size_t m;
	for (m = 0; m < length; m++) {
		const long double *b_macc = b;
		const long double *in_macc = in;
		const long double *a_macc = a;
		const long double *out_macc = out;
		long double b_acc = (*in_macc--) * (*b_macc++);
		long double a_acc = 0;
		while (a_macc <= a_end && out_macc >= out_start) {
			b_acc += (*in_macc--) * (*b_macc++);
			a_acc += (*out_macc--) * (*a_macc++);
		}
		*++out = (b_acc - a_acc) / a0;
		in++;
	}
}

uint16_t ntohs(uint16_t netshort) {
	return ((netshort & 0xff) << 8) | (netshort >> 8);
}

uint32_t be_bytes_to_address(uint8_t *address_bytes)
{
	uint32_t address = 0;
	uint8_t *addr_ptr = (uint8_t *) &address;
	for (int32_t x = 0; x < 3; x++) {
		addr_ptr[x] = address_bytes[2 - x];
	}

	return address;
}

int32_t parse_control_rom(FILE *f, struct instrument *instruments, struct part *parts,
	struct sample *samples, struct variation *variations, struct drum *drums,
	uint32_t *banks, bool sc55)
{
	int32_t part_index = 0;
	int32_t instrument_index = 0;
	int32_t sample_index = 0;

	int32_t instrument_count = 0;

	int32_t step = B1_SZ;
	fseek(f, banks[0], SEEK_SET);
	for (int32_t x = banks[0]; x < banks[6]; x += step) {

		if ((x & 0xFFFF) == (banks[0] & 0xFFFF)) step = B1_SZ;
		else if ((x & 0xFFFF) == (banks[1] & 0xFFFF)) step = B2_SZ;
		else if ((x & 0xFFFF) == (banks[2] & 0xFFFF)) step = B3_SZ;

		if (step == B3_SZ) {
			fread(&samples[sample_index], 1, step, f);
			if (sc55) {
				samples[sample_index].loop_len= ntohs(samples[sample_index].loop_len);
				samples[sample_index].sample_len = ntohs(samples[sample_index].sample_len);
				samples[sample_index].attack_end = ntohs(samples[sample_index].attack_end);
				samples[sample_index].pitch = ntohs(samples[sample_index].pitch);
				samples[sample_index].fine_volume = ntohs(samples[sample_index].fine_volume);
			}
			// printf("Sample %3d: V:%3d U1:%5d SL:%5d LL:%5d LM:%3d RK:%3d FP:%+5d FV:%+5d\n",
			// sample_index,
			// samples[sample_index].volume,
			// samples[sample_index].attack_end,
			// samples[sample_index].sample_len,
			// samples[sample_index].loop_len,
			// samples[sample_index].loop_mode,
			// samples[sample_index].root_key,
			// samples[sample_index].pitch - 1024,
			// samples[sample_index].fine_volume - 1024);
			sample_index++;
		} else if (step == B1_SZ) {
			fread(&instruments[instrument_index], 1, step, f);
			if (sc55) {
				instruments[instrument_index].parts[0].part_index = ntohs(instruments[instrument_index].parts[0].part_index);
				instruments[instrument_index].parts[1].part_index = ntohs(instruments[instrument_index].parts[1].part_index);
			}
			if (instruments[instrument_index].name[0])
				instrument_count++;
			instrument_index++;
		} else {
			fread(&parts[part_index], 1, step, f);
			for (int32_t sflip = 0; sflip < 16; sflip++) {
				if (sc55) {
					parts[part_index].samples[sflip] = ntohs(parts[part_index].samples[sflip]);
				}
			}
			part_index++;
		}
	}

	fseek(f, banks[6], SEEK_SET);

	for (int32_t x = 0; x < 128; x++) {
		fread(variations[x].variation, B4_SZ, 1, f);
		if (sc55) {
			for (int32_t y = 0; y < 128; y++){
				uint8_t *b = (uint8_t *) &variations[x].variation[y];
				uint8_t top = b[0];
				b[0] = b[1];
				b[1] = top;
			}
		}
	}

	// Drums
	if (sc55) {
		uint32_t index = 0;
		for (int32_t x = banks[7]; x < 0x3c028; x += KIT_SIZE) {
			fseek(f, x, SEEK_SET);
			fread(&drums[index], sizeof(struct drum), 1, f);
			for (int32_t y = 0; y < 128; y++){
				uint8_t *b = (uint8_t *) &drums[index].preset[y];
				uint8_t top = b[0];
				b[0] = b[1];
				b[1] = top;
			}
			index++;
		}
	}

	return instrument_count;
}

void *trim_name(char *name)
{
	for (int32_t x = NAME_SZ - 1; x >= 0; x--) {
		if (isspace(name[x])) {
			name[x] = '\0';
		} else {
			break;
		}
	}
}

void clean_name(char *dirty_name, char *name)
{
	// Not all names are NULL terminated, so this returns them safely
	memset(name, 0, NAME_SZ + 1);
	memcpy(name, dirty_name, NAME_SZ);
	trim_name(name);
}

uint32_t unscramble_address(uint32_t address)
{
	// Discovered and written by NewRisingSun
	uint32_t new_addr = 0;
	if (address >= 0x20) {	// The first 32 bytes are not encrypted
		static const int addressOrder [20] = {0x02, 0x00, 0x03, 0x04, 0x01, 0x09, 0x0D, 0x0A, 0x12,
			0x11, 0x06, 0x0F, 0x0B, 0x10, 0x08, 0x05, 0x0C, 0x07, 0x0E, 0x13};
		for (uint32_t bit = 0; bit < 20; bit++) {
			new_addr |= ((address >> addressOrder[bit]) & 1) << bit;
		}
	} else {
		new_addr = address;
	}

	return new_addr;
}

int8_t unscramble_byte(int8_t byte)
{
	uint8_t byte_order[8] = {2, 0, 4, 5, 7, 6, 3, 1};
	uint32_t new_byte = 0;

	for (uint32_t bit = 0; bit < 8; bit++) {
		new_byte |= ((byte >> byte_order[bit]) & 1) << bit;
	}

	return new_byte;
}

bool decode_wave_rom(uint8_t *dec_buf)
{
	char *files_in[3] = {"roms"PATH_DIV"roland-gss.a_r15209276.ic28", "roms"PATH_DIV"roland-gss.b_r15209277.ic27", "roms"PATH_DIV"roland-gss.c_r15209281.ic26"};

	uint8_t *enc_buf = calloc(1, 0x100000);

	for (int32_t x = 0; x < 3; x++) {
		FILE *f_in = fopen(files_in[x], "rb");
		if (!f_in) {
			printf("Unable to find wave roms. Results will be corrupt.\n");
			return false;
		}
		fread(&enc_buf[0], 1, 0x100000, f_in);
		fclose(f_in);
		for (uint32_t y = 0; y < 0x100000; y++) {
			dec_buf[unscramble_address(y) + (0x100000 * x)] = unscramble_byte(enc_buf[y]);
		}
	}

	FILE *fo = fopen("wave_dec.rom", "wb");
	fwrite(dec_buf, 0x300000, 1, fo);
	fclose (fo);

	free(enc_buf);
	return true;
}

bool decode_wave_rom_scb(uint8_t *dec_buf)
{
	uint8_t *enc_buf = calloc(1, 0x200000);

	FILE *f_in = fopen("roms"PATH_DIV"R15209359_(samples1).BIN", "rb");

	if (!f_in) {
		printf("Unable to find wave roms. Results will be corrupt.\n");
		return false;
	}
	fread(&enc_buf[0], 1, 0x200000, f_in);
	fclose(f_in);
	for (uint32_t y = 0; y < 0x100000; y++) {
		dec_buf[unscramble_address(y) + (0x000000)] = unscramble_byte(enc_buf[y]);
	}

	for (uint32_t y = 0; y < 0x100000; y++) {
		dec_buf[unscramble_address(y) + (0x100000)] = unscramble_byte(enc_buf[y + (0x100000)]);
	}

	FILE *fo = fopen("wave_dec_scb1.rom", "wb");
	fwrite(dec_buf, 0x200000, 1, fo);
	fclose (fo);

	f_in = fopen("roms"PATH_DIV"R15279813_(samples2).BIN", "rb");
	if (!f_in) {
		printf("Unable to find wave roms. Results will be corrupt.\n");
		return false;
	}
	fread(&enc_buf[0], 1, 0x100000, f_in);
	fclose(f_in);
	for (uint32_t y = 0; y < 0x100000; y++) {
		dec_buf[unscramble_address(y) + (0x200000)] = unscramble_byte((enc_buf[y]));
	}

	fo = fopen("wave_dec_scb2.rom", "wb");
		fwrite(dec_buf, 0x100000, 1, fo);
	fclose (fo);

	free(enc_buf);
	return true;
}

//FIXME: Unused
struct instrument *get_instrument(struct synth *synth, uint16_t midi_number, uint16_t bank)
{
	struct instrument *inst = NULL;
	if (midi_number > 127 || bank > NUM_VARIATIONS - 1) {
		printf ("rejecting %d %d\n", midi_number, bank);
		return inst;
	}

	if (synth->variations[bank].variation[midi_number] < NUM_INST) {
		if (synth->instruments[synth->variations[bank].variation[midi_number]].name[0]) {
			inst = &synth->instruments[synth->variations[bank].variation[midi_number]];
		} else {
			printf("Nameless instrument at %04x\n", synth->variations[bank].variation[midi_number]);
		}
	} else if (synth->variations[bank].variation[midi_number] != 0xFFFF) {
		printf("Unknown inst: %04x\n", synth->variations[bank].variation[midi_number]);
	}

	return inst;
}

//FIXME: Unused
struct part *get_part(struct synth *synth, struct instrument *inst, uint8_t part_num)
{
	struct part *part = NULL;
	if (inst->parts[part_num].part_index < NUM_PARTS &&
		synth->parts[inst->parts[part_num].part_index].name[0])
		part = &synth->parts[inst->parts[part_num].part_index];
	else
	{
		printf("Nameless part at %04x\n", inst->parts[part_num].part_index);
	}

	return part;
}

//FIXME: Unused
struct sample *get_sample(struct synth *synth, struct part *part, uint8_t sample_index)
{
	struct sample *sample = NULL;
	if (sample_index > 15)
		return sample;

	if (part->samples[sample_index] < NUM_SAMPLES &&
		synth->samples[part->samples[sample_index]].sample_len > 0) {
		sample = &synth->samples[part->samples[sample_index]];
	}

	return sample;
}

//FIXME: Unused
void print_bits(uint32_t bits, size_t num_bits)
{
	for (int32_t x = num_bits - 1; x >=0; x--) {
		if (x % 4 == 3)
			printf(" ");
		if (bits & (1 << x))
			printf("1");
		else
			printf("0");
	}
}

void export_sample(int32_t *samples, size_t num_samples, char *file_name)
{
	struct RIFF *samp = NULL;
	riff_open(&samp, "WAVE", file_name);
	struct wav_params p = {0};
	p.audio_format = 1;
	p.num_channels = 1;
	p.sample_rate = 32000;
	p.bits_per_sample = 24;
	p.byte_rate = p.sample_rate * p.num_channels * p.bits_per_sample / 8;
	p.block_align = p.num_channels * p.bits_per_sample / 8;

	riff_write_chunk(samp, "fmt ", sizeof(struct wav_params), (uint8_t *)&p);

	uint8_t *sample_24 = calloc(1, num_samples * 3 * sizeof(uint8_t));

	for (int32_t x = 0; x < num_samples; x++) {
		memcpy(&sample_24[x * 3], &samples[x], 3);
	}

	riff_write_chunk(samp, "data", num_samples * 3, sample_24);
	riff_close(&samp);

	free(sample_24);
}

uint32_t apply_riaa_filter(biquad bq, int32_t *sample, size_t len, double vol, uint32_t sample_num)
{
	long double attenuate = vol / (SC552AMP(127));

	long double *conv_buffer = calloc(len + 1000, sizeof(long double));
	long double *conv_buffer2 = calloc(len + 1000, sizeof(long double));

	for(int32_t x = 0; x < len; x++) {
		conv_buffer[x] = sample[x];
	}

	biquad hpf;
	biquad lpf;
	// hpf.d[0] = 1.000000000000000L;
	// hpf.d[1] = -0.999018733894812L;
	// hpf.n[0] = 0.999509366947406L;
	// hpf.n[1] = -0.999509366947406l;
	// hpf.nd = 2;
	// hpf.nn = 2;

	hpf.d[0] = 1.000000000000000L;
	hpf.d[1] = -0.998038429728411L;
	hpf.n[0] = 0.999019214864206L;
	hpf.n[1] = -0.999019214864206l;
	hpf.nd = 2;
	hpf.nn = 2;

	// Samples use heavy pre-emphasis. RIAA curve x2 appears to make them sound normal.
	iir_filter(&bq.n[0], &bq.d[0], bq.nd, conv_buffer, conv_buffer2, len);
	iir_filter(&bq.n[0], &bq.d[0], bq.nd, conv_buffer2, conv_buffer, len);

	// Apply a HPF to remove bias from the audio
	iir_filter(&hpf.n[0], &hpf.d[0], hpf.nd, conv_buffer, conv_buffer2, len);

	size_t clipped = 0;
	for (int32_t x = 0; x < len; x++) {
		// FIXME: Unfortunately after filtering twice the volume is extremely low, and the range
		// much too high so I do a raw shift of * 8 and then use a compressor to further increase
		// the percieved volume.
		int32_t t_val = compress_sample((conv_buffer2[x] * 4.0L) * attenuate, 16);

		if (t_val > INT24_MAX) {
			t_val = INT24_MAX;
			clipped++;
		} else if (t_val < INT24_MIN) {
			t_val = INT24_MIN;
			clipped++;
		}

		sample[x] = round(t_val);
	}

	if (clipped)
		printf("%ld samples clipped in sample %d\n", clipped, sample_num);

	free(conv_buffer);
	free(conv_buffer2);

	return len;
}

int32_t make_sample(uint8_t *decoded_rom, uint32_t address)
{
	int8_t data_byte = decoded_rom[address];
	uint8_t shift_byte = decoded_rom[((address & 0xFFFFF) >> 5) | (address & 0xF00000)];
	uint8_t shift_nibble = (address & 0x10) ? (shift_byte >> 4 ) : (shift_byte & 0x0F);
	int32_t final = ((data_byte << shift_nibble) << 14); // Shift nibbles thus far never exceed 10, thus 18 bit samples
	final = final >> 8; // To maintain sign for 24 bit sample

	return final;
}

uint32_t write_sample_data(uint32_t address, uint32_t loop_start, uint32_t loop_mode, uint32_t loop_end, int32_t length,
	int32_t *delta, int32_t *samples, size_t *data_size, uint8_t *decoded_rom, uint32_t sample_end)
{
	if (length <= 0) {
		printf("Invalid Length %d written\n", length);
		return address;
	}

	// Delta is used for loop mode 1 forward-then-back looping. Set 0 for mode 0.
	switch (loop_mode) {
		case 0:
			while (length--) {
				samples[*data_size] = make_sample(decoded_rom, address++);
				*data_size = *data_size + 1;
				if (address == loop_end) address = loop_start;
			}
		break;

		case 1:
			while (length--) {
				samples[*data_size] = make_sample(decoded_rom, address);
				*data_size = *data_size + 1;
				address += *delta;
				if (address > loop_end) {
					*delta = -1;
					address--;
				} else if (address == loop_start && *delta < 0) {
					*delta = 1;
					address++;
				}
			}
		break;

		case 2:
			while (length--) {
				if (address > sample_end) {
					printf("Sample writing exceeded end of sample in non-looping sample by %d bytes.\n", length);
					return address;
				}
				samples[*data_size] = make_sample(decoded_rom, address++);
				*data_size = *data_size + 1;
			}
		break;
	}

	return address;
}

void apply_envelope(int32_t *samples, uint32_t length, double *initial_amplitude, double target_amplitude, uint8_t shape)
{
	if (length <= 0) return;
	double current_amp = *initial_amplitude;
	int32_t total_length = length;
	double max_amp = SC552AMP(0X7F);
	double level_difference = (target_amplitude - *initial_amplitude);
	uint32_t samp_index = 0;
	while (1) {
		samp_index++;
		if (samp_index > length) break;
		switch (shape) {
			case 0: // Linear
				current_amp = *initial_amplitude + level_difference * ((double)samp_index / (double)length);
			break;

			case 1: // Concave
			case 2: // "Convex"
				current_amp = *initial_amplitude + level_difference * (log(10.0 *samp_index / length + 1) / log(10.0 + 1));
			break;
			
			case 4: // Level only
				current_amp = target_amplitude;
			break;
			
		}
		samples[samp_index] = round((double)samples[samp_index] * (current_amp / max_amp));
	}

	*initial_amplitude = current_amp;
}

#define MIN_SAMPLE_PAD 512
#define TIME2SAMP(x) ((int32_t)(round(32000.0 * (double)(x))))

uint32_t fill_single_sample(struct sf_samples *s, struct sample *sc55_samples, uint8_t *dec,
	uint32_t source, struct sample_params *params)
{
	// Also, to allow a variety of hardware platforms to be able to reproduce the data, the samples
	// have a minimum length of 48 data points, a minimum loop size of 32 data points and a minimum
	// of 8 valid points prior to dwStartloop and after dwEndloop. Thus dwStart must be less than
	// dwStartloop-7, dwStartloop must be less than dwEndloop-31, and dwEndloop must be less than
	// dwEnd-7. If these constraints are not met, the sound may optionally not be played if the
	// hardware cannot support artifact-free playback for the parameters given.

	// Additionally, each sample must be followed by at least 46 zero-valued data points

	biquad bq = compute_riaa_irr(32000.0, 9.89808, 1);

	uint32_t sample_address = be_bytes_to_address(sc55_samples[source].offset);
	uint32_t bank = 0;
	switch ((sample_address & 0x700000) >> 20) {
		case 0: bank = 0x000000; break;
		case 1: bank = 0x100000; break; // Used in SCB/MKII
		case 2: bank = 0x100000; break;
		case 4: bank = 0x200000; break;
		default: printf("Encountered unknown bank ID: %d\n", (sample_address & 0x700000));
	}
	uint32_t address = (sample_address & 0xFFFFF) | bank;
	uint32_t total_length = sc55_samples[source].sample_len + 1;
	uint32_t loop_length = sc55_samples[source].loop_len;

	uint32_t loop_start_offset = total_length - 2 - loop_length;
	uint32_t loop_end_offset = total_length - 1;
	int32_t loop_start = address + loop_start_offset;
	int32_t loop_end = address + loop_end_offset;

	char sample_name[40] = {0};
	snprintf(s->shdr[s->num_samples].achSampleName, 20, "SC-55 - %d", source);

	s->shdr[s->num_samples].byOriginalPitch = sc55_samples[source].root_key;
	s->shdr[s->num_samples].dwSampleRate = 32000;
	s->shdr[s->num_samples].sfSampleType = 1;
	s->shdr[s->num_samples].wSampleLink = 0;
	s->shdr[s->num_samples].chPitchCorrection = round(((double)sc55_samples[source].pitch - 1024.0) / 10.0);
	s->shdr[s->num_samples].dwStart = s->data_size;
	s->shdr[s->num_samples].dwStartloop = s->shdr[s->num_samples].dwStart + loop_start_offset + 2;
	s->shdr[s->num_samples].dwEndloop = s->shdr[s->num_samples].dwStart + loop_end_offset;

	// 30 second buffer size
	int32_t *sbuf_full = calloc(120 * 32000, sizeof(int32_t));
	int32_t *sbuf = &sbuf_full[2 * 32000];
	size_t sbuf_size = 0;
	uint32_t sample_end = address + total_length;

	int32_t delta = 1;
	uint32_t addr_ptr = address;
	uint32_t real_end = 0;

	// Phase 1, Phase 2, and Phase 5 always exist. The other two are optional. However, because the
	// terminal phase has the potential to be very long, for the purposes of space, it would be best
	// to only account for the phases prior to it. In some situations, like Seashore, the sample
	// may end up being extremely long regardless.

	uint32_t env_index = 0;
	size_t starts[5] = {0};
	double levels[5] = {0};
	uint8_t shapes[5] = {0};

	uint32_t env_start = 0;
	double initial_amplitude = 0.000001;
	// Phase 1 (Attack)
	addr_ptr = write_sample_data(addr_ptr, loop_start, sc55_samples[source].loop_mode, loop_end,
		TIME2SAMP(params->t1), &delta, sbuf, &sbuf_size, dec, sample_end);
	levels[env_index] = params->l1;
	shapes[env_index] = params->s1;
	starts[env_index++] = sbuf_size;

	// Phase 2
	if (params->terminal_phase > 2) {
		addr_ptr = write_sample_data(addr_ptr, loop_start, sc55_samples[source].loop_mode, loop_end,
			TIME2SAMP(params->t2), &delta, sbuf, &sbuf_size, dec, sample_end);
		levels[env_index] = params->l2;
		shapes[env_index] = params->s2;
		starts[env_index++] = sbuf_size;
	}

	// Phase 3
	if (params->terminal_phase > 3) {
		addr_ptr = write_sample_data(addr_ptr, loop_start, sc55_samples[source].loop_mode, loop_end,
			TIME2SAMP(params->t3), &delta, sbuf, &sbuf_size, dec, sample_end);
		levels[env_index] = params->l3;
		shapes[env_index] = params->s3;
		starts[env_index++] = sbuf_size;
	}

	double final_level = 0;
	switch (params->terminal_phase) {
		case 2: final_level = params->l1; break;
		case 3: final_level = params->l2; break;
		case 4: final_level = params->l3; break;
	}

	if (sc55_samples[source].loop_mode != 2) {
		// Square off sample to the nearest loop
		addr_ptr = write_sample_data(addr_ptr, loop_start, sc55_samples[source].loop_mode, loop_end,
			delta < 0 ? loop_length + (loop_end - addr_ptr) : (loop_end - addr_ptr), &delta, sbuf, &sbuf_size, dec, sample_end);
		s->shdr[s->num_samples].dwStartloop = s->data_size + sbuf_size; //FIXME: - 1?
		addr_ptr = write_sample_data(addr_ptr, loop_start, sc55_samples[source].loop_mode, loop_end,
			(loop_end - loop_start) * (sc55_samples[source].loop_mode == 1 ? 2 : 1), &delta, sbuf, &sbuf_size, dec, sample_end);
		s->shdr[s->num_samples].dwEndloop = s->data_size + sbuf_size;

		real_end = s->data_size + 8;
		// Write a large tail so that the RIAA filter doesn't mis-align the amplitude of loop-ends
		addr_ptr = write_sample_data(addr_ptr, loop_start, sc55_samples[source].loop_mode, loop_end,
			MIN_SAMPLE_PAD, &delta, sbuf, &sbuf_size, dec, sample_end);
		levels[env_index] = final_level;
		shapes[env_index] = 4;
		starts[env_index++] = sbuf_size;

	} else {
		// Write any remaining data for non-looping samples
		addr_ptr = write_sample_data(addr_ptr, loop_start, sc55_samples[source].loop_mode, loop_end,
			sample_end - addr_ptr, &delta, sbuf, &sbuf_size, dec, sample_end);
		levels[env_index] = final_level;
		shapes[env_index] = 4;
		starts[env_index++] = sbuf_size;
		real_end = sbuf_size;
	}

	s->shdr[s->num_samples].dwEnd = s->data_size + sbuf_size;

	uint32_t len = s->shdr[s->num_samples].dwEnd - s->shdr[s->num_samples].dwStart;

	double volume = SC552AMP((double)sc55_samples[source].volume) + (((double)(sc55_samples[source].fine_volume - 1024) / 1000.0));

	uint32_t last_value = 0;
	for (int32_t x = 0; x < 5; x++) {
		if (starts[x]) {
			apply_envelope(&sbuf[last_value], starts[x] - last_value, &initial_amplitude, levels[x], shapes[x]);
			last_value = starts[x];
		}
	}

	// Add data to prevent filter nonsense
	for (int32_t x = 0; x < (2 * 32000); x++) {
		sbuf_full[(2 * 32000) - x] = sbuf_full[(2 * 32000) + x];
	}
	apply_riaa_filter(bq, sbuf_full, len + (2 * 32000), volume, source);
	
	for (int32_t x = 0; x < sbuf_size; x++) {
		s->sample16[s->data_size + x] = sbuf[x] >> 8;
		s->sample8[s->data_size + x] = (sbuf[x] & 0xFF);
	}

	free(sbuf_full);
	s->data_size += sbuf_size + 46;
	if (s->shdr[s->num_samples].dwStartloop - s->shdr[s->num_samples].dwStart < 8) {
		printf("Short pre-loop sample %d\n", source);
	}
	if (s->shdr[s->num_samples].dwEnd - s->shdr[s->num_samples].dwStart < 48) {
		printf("Short sample %d\n", source);
	}

	return s->num_samples++;
}


uint32_t find_or_make_sample(struct sf_samples *s, struct sample *sc55_samples, uint8_t *dec,
	uint32_t source, struct sample_params *params)
{
	uint32_t first_unused = 0;
	for (int32_t x = 0; x < 12; x++) {
		if (s->used[source][x].used) {
			if (!memcmp(&s->used[source][x].params, params, sizeof(struct sample_params)))
				return s->used[source][x].sample_number;
		} else {
			first_unused = x;
			break;
		}
	}
	uint32_t new_sample = fill_single_sample(s, sc55_samples, dec, source, params);
	s->used[source][first_unused].used = true;
	s->used[source][first_unused].sample_number = new_sample;
	memcpy(&s->used[source][first_unused].params, params, sizeof(struct sample_params));

	return new_sample;
}

#define SEC2SF(x) ((x) ? 1200.0 * log2(x) : INT16_MIN)
#define CONV_VALUE(x) ((pow(2.0, (double)(x & 0x7F) / 18.0) / 5.45 - 0.183))

void add_igen_short(struct sf_instruments *i, uint16_t operator, int32_t value)
{
	int16_t clean_value = value < INT16_MIN ? INT16_MIN : value > INT16_MAX ? INT16_MAX : value;
	i->igen[i->igen_count].sfGenOper = operator;
	i->igen[i->igen_count++].genAmount.shAmount = clean_value;
}

void add_igen_word(struct sf_instruments *i, uint16_t operator, int32_t value)
{
	uint16_t clean_value = value < 0 ? 0 : value > UINT16_MAX ? UINT16_MAX : value;
	i->igen[i->igen_count].sfGenOper = operator;
	i->igen[i->igen_count++].genAmount.wAmount = value;
}

void add_igen_split(struct sf_instruments *i, uint16_t operator, uint8_t max, uint8_t min)
{
	i->igen[i->igen_count].sfGenOper = operator;
	i->igen[i->igen_count].genAmount.ranges.byLo = min;
	i->igen[i->igen_count++].genAmount.ranges.byHi = max;
}

void add_imod(struct sf_instruments *i, uint16_t operator, uint16_t trans_operator,
	uint16_t dest_operator, int16_t amount, uint16_t amount_source)
{
	i->imod[i->imod_count].sfModSrcOper = operator;
	i->imod[i->imod_count].sfModDestOper = dest_operator;
	i->imod[i->imod_count].modAmount = amount;
	i->imod[i->imod_count].sfModAmtSrcOper = amount_source;
	i->imod[i->imod_count++].sfModTransOper = trans_operator;
}

void add_pmod(struct sf_presets *p, uint16_t operator, uint16_t trans_operator,
	uint16_t dest_operator, int16_t amount, uint16_t amount_source)
{
	p->pmod[p->pmod_count].sfModSrcOper = operator;
	p->pmod[p->pmod_count].sfModTransOper = trans_operator;
	p->pmod[p->pmod_count].sfModDestOper = dest_operator;
	p->pmod[p->pmod_count].modAmount = amount;
	p->pmod[p->pmod_count++].sfModAmtSrcOper = amount_source;
}

#define FIRST_VAL 0.5
#define LAST_VAL 1.0

#define MAX_FILTER 16000.0
#define HZ2CENT(x) (round(log((double)(abs(x)) / 440.0) / log(2.0) * 1200.0 + 6900.0) * ((x) < 0 ? -1.0 : 1.0))
#define SC552HZ(x) round(((double)(/*0x7F -*/ ((x) & 0x7F)) / 127.0) * MAX_FILTER)
void add_instrument_params(struct ins_partial *p, struct sf_instruments *i, struct instrument *inst, bool is_drum,
	struct drum *drum, uint32_t drum_index, struct synth *sc55, struct sample_params *params)
{
	//-------------------//
	//  Volume Envelope  //
	//-------------------//

	double value = 0;

	double p1_val = CONV_VALUE(p->pp[pp_tva_p1_len]);
	double p2_val = CONV_VALUE(p->pp[pp_tva_p2_len]);
	double p3_val = CONV_VALUE(p->pp[pp_tva_p3_len]);
	double p4_val = CONV_VALUE(p->pp[pp_tva_p4_len]);

	value = CONV_VALUE(p->pp[pp_tva_p5_len]);
	value /= (p->pp[pp_tva_p5_len] & 0x80) ? FIRST_VAL : LAST_VAL;
	double p5_val = value;

	double p1_vol = SC552AMP(p->pp[pp_tva_p1_vol]);
	double p2_vol = SC552AMP(p->pp[pp_tva_p2_vol]);
	double p3_vol = SC552AMP(p->pp[pp_tva_p3_vol]);
	double p4_vol = SC552AMP(p->pp[pp_tva_p4_vol]);

	params->l1 = p1_vol;
	params->l2 = p2_vol;
	params->l3 = p3_vol;
	params->l4 = p4_vol;

	params->t1 = p1_val;
	params->t2 = p2_val;
	params->t3 = p3_val;
	params->t4 = p4_val;
	params->t5 = p5_val;

	params->s1 = (p->pp[pp_tva_p1_len] & 0x80) ? 0 : 1;
	params->s2 = (p->pp[pp_tva_p2_len] & 0x80) ? 0 : 1;
	params->s3 = (p->pp[pp_tva_p3_len] & 0x80) ? 0 : 1;
	params->s4 = (p->pp[pp_tva_p4_len] & 0x80) ? 0 : 1;
	params->s5 = (p->pp[pp_tva_p5_len] & 0x80) ? 0 : 1;

	uint8_t terminal_atten = 0x7F;

	if (p->pp[pp_tva_p2_vol] == 0) {
		params->terminal_phase = 2;
	} else if (p->pp[pp_tva_p3_vol] == 0) {
		params->terminal_phase = 3;
	} else if (p->pp[pp_tva_p3_vol] == p->pp[pp_tva_p2_vol] && p->pp[pp_tva_p3_vol] == p->pp[pp_tva_p4_vol]) {
		params->terminal_phase = 2;
	} else if(p->pp[pp_tva_p3_vol] == p->pp[pp_tva_p4_vol]) {
		params->terminal_phase = 3;
	} else {
		params->terminal_phase = 4;
	}

	char name[20];
	clean_name(inst->name, name);
	bool no_loops = sc55->samples[sc55->parts[p->part_index].samples[0]].loop_mode == 2;
	// if (is_drum) {
	// 	printf("%12s: T1:%2.3f T2:%2.3f T3:%2.3f T4:%2.3f T5:%2.3f || L1:%2.3f L2:%2.3f L3:%2.3f L4:%2.3f Loop:%d NOF:%d NON:%d\n",
	// 	name, p1_val, p2_val, p3_val, p4_val, p5_val,
	// 	p1_vol, p2_vol, p3_vol, p4_vol,
	// 	!no_loops, (drum->flags[drum_index] & 0x01) > 0, (drum->flags[drum_index] & 0x10) > 0);
	// } else {
	// 	printf("%12s: T1:%2.3f T2:%2.3f T3:%2.3f T4:%2.3f T5:%2.3f || L1:%2.3f L2:%2.3f L3:%2.3f L4:%2.3f\n",
	// 	name, p1_val, p2_val, p3_val, p4_val, p5_val,
	// 	p1_vol, p2_vol, p3_vol, p4_vol);
	// }

	double decay = 0;
	double hold = 0;
	double release = 0;

	switch (params->terminal_phase) {
		case 2:
			hold = p1_val;
			decay = !params->s2 ? p2_val * 2.0 : p2_val;
			terminal_atten = p->pp[pp_tva_p2_vol];
			break;

		case 3:
			hold = p1_val + p2_val;
			decay = !params->s3 ? p3_val * 2.0 :p3_val;
			terminal_atten = p->pp[pp_tva_p3_vol];
			break;

		default:
			hold = p1_val + p2_val + p3_val;
			decay = !params->s4 ? p4_val * 2.0 : p4_val;
			terminal_atten = p->pp[pp_tva_p4_vol];
			break;
	}

	bool max_sustain = false;
	if ((is_drum && !(drum->flags[drum_index] & 0x01))) { // Simulate ignore note_off
		if (no_loops) {
			release = (hold + decay) + 8.0;
		} else {
			release = hold + decay;
			decay = release;
			hold = 0;
		}
		max_sustain = true;
	} else {
		release = p5_val;
	}

	add_igen_word(i, sfg_releaseVolEnv, release ? SEC2SF(release) : INT16_MIN);
	add_igen_word(i, sfg_holdVolEnv, hold ? SEC2SF(hold) : INT16_MIN);
	add_igen_word(i, sfg_decayVolEnv, decay ? SEC2SF(decay) : INT16_MIN);


	if (is_drum)
		add_imod(i, 0x00DB, 0, 0x0010, (((double)drum->reverb[drum_index]/ 127.0) * (double)inst->header[ih_reverb]) * 10, 0);
	else
		add_imod(i, 0x00DB, 0, 0x0010, inst->header[ih_reverb] * 10, 0);

	if (is_drum)
		add_imod(i, 0x00DD, 0, 0x000F, (((double)drum->chorus[drum_index]/ 127.0) * (double)inst->header[ih_chorus]) * 10, 0);
	else
		add_imod(i, 0x00DD, 0, 0x000F, inst->header[ih_chorus] * 10, 0);

	if(max_sustain) {
		add_igen_word(i, sfg_sustainVolEnv, 1440);
	} else {
		add_igen_word(i, sfg_sustainVolEnv, round(PCT2VOL(terminal_atten)));
	}

	if (is_drum)
		add_igen_word(i, sfg_initialAttenuation, round(PCT2VOL(p->pp[pp_part_attenuation] - (0x7f - drum->volume[drum_index])))); // FIXME: drums have volume mod
	else
		add_igen_word(i, sfg_initialAttenuation, round(PCT2VOL(p->pp[pp_part_attenuation])));

	// --------------------//
	// Modulator Envelope //
	// --------------------//

	// FIXME: Currently experimental, uncomment to use

	/*
	double attack_mod = CONV_VALUE(p->pp[46]);
	double hold_mod = CONV_VALUE(p->pp[47]);
	double decay_mod = CONV_VALUE(p->pp[48]) + CONV_VALUE(p->pp[49]) + hold_mod;
	double release_mod = CONV_VALUE(p->pp[50]);

	add_igen_word(i, sfg_attackModEnv, SEC2SF(attack_mod));
	//add_igen_word(i, sfg_holdModEnv, SEC2SF(hold_mod));
	add_igen_word(i, sfg_decayModEnv, SEC2SF(decay_mod));
	add_igen_word(i, sfg_releaseModEnv, SEC2SF(release_mod));

	double base_filter = SC552HZ(p->pp[40]);
	double initial_filter = (((double)(p->pp[41]) / 127.0) * base_filter);
	double terminal_filter = (((double)(p->pp[45]) / 127.0) * base_filter);
	double sustain_filter = (1.0 - (terminal_filter / base_filter) * 1000.0);

	if (p->pp[40]) {
		add_igen_short(i, sfg_modEnvToFilterFc, HZ2CENT(initial_filter)); // Number in Cents, can be negative
		add_igen_word(i, sfg_sustainModEnv, sustain_filter); // Number in 0.1% units
		add_igen_word(i, sfg_initialFilterFc, 0); // Number in Cents
	}

	// if (p->pp[34]) {
	// 	add_igen_word(i, sfg_initialFilterQ, HZ2CENT(base filter - SC552HZ(p->pp[34])));
	// }
	*/

	//---------------//
	//  Other Stuff  //
	//---------------//

	add_igen_short(i, sfg_coarseTune, p->pp[pp_course_pitch] - 0x40);
	add_igen_short(i, sfg_fineTune, round((double)(p->pp[pp_fine_pitch] - 0x40)));

	if(!is_drum) {
		if (p->pp[pp_panpot])
			add_igen_short(i, sfg_pan, ((double)(p->pp[pp_panpot] - 0x40) / 64.0) * -500.0);
	} else {
		if (drum->panpot[drum_index])
			add_igen_short(i, sfg_pan, ((double)(drum->panpot[drum_index] - 0x40) / 64.0) * -500.0);
	}

	add_igen_word(i, sfg_reverbEffectsSend, 70);

	// if (p->pp[pp_vibrato_depth]) { //FIXME: Find scales
	// 	add_igen_word(i, sfg_vibLfoToPitch, PCT2VOL(0x7F - (p->pp[pp_vibrato_depth] & 0x7F)));
	// 	//add_igen_word(i, sfg_delayVibLFO, SEC2SF(CONV_VALUE(p->pp[11])));
	// 	//add_igen_short(i, sfg_freqVibLFO, (double)(p->pp[55] - 0x40) * 100.0);
	// }

	// if (p->pp[pp_lfo_depth]) { //FIXME: Find scales
	// 	add_igen_word(i, sfg_modLfoToVolume, PCT2VOL(0x7F - (p->pp[pp_lfo_depth] & 0x7F)));
	// 	//add_igen_word(i, sfg_delayModLFO, SEC2SF(CONV_VALUE(p->pp[69])));
	// 	//add_igen_short(i, sfg_freqModLFO, (double)(p->pp[56] - 0x40) * 100);
	// }

	if (!is_drum) {
		add_igen_word(i, sfg_keynumToVolEnvDecay, 0);
		add_igen_word(i, sfg_keynumToVolEnvHold, 0);
		add_igen_word(i, sfg_keynumToModEnvDecay, 0);
		add_igen_word(i, sfg_keynumToModEnvHold, 0);
	} else {
		add_igen_word(i, sfg_keynumToVolEnvDecay, 0);
		add_igen_word(i, sfg_keynumToVolEnvHold, 0);
		add_igen_word(i, sfg_keynumToModEnvDecay, 0);
		add_igen_word(i, sfg_keynumToModEnvHold, 0);
	}


}

uint32_t add_instrument(struct synth *sc55, struct sf_samples *s, uint32_t part_index,
	struct ins_partial *partial, struct sf_instruments *i, struct instrument *inst)
{
	struct part *part = &sc55->parts[part_index];

	if (part->name[0]) {
		char name [NAME_SZ + 1] = {0};
		clean_name(part->name, name);
		if (i->used_inst[part_index]) {
			snprintf(i->inst[i->inst_count].achInstName, 20, "%s - %d", name, i->used_inst[part_index]);
		} else {
			snprintf(i->inst[i->inst_count].achInstName, 20, "%s", name);
		}
		i->used_inst[part_index]++;

		uint32_t y = 0;
		uint8_t last_value = 0;
		i->inst[i->inst_count].wInstBagNdx = i->ibag_count;
		i->ibag[i->ibag_count].wInstGenNdx = i->igen_count;
		i->ibag[i->ibag_count].wInstModNdx = i->imod_count;

		struct sample_params params = {0};
		add_instrument_params(partial, i, inst, false, NULL, 0, sc55, &params);
		// if (partial->pp[74] * 0x80) {
		// 	printf("adding modulator for %s\n", name);
		// 	i->ibag[i->ibag_count].wInstModNdx = i->imod_count;
		// 	i->imod[i->imod_count].sfModSrcOper = 0x0502;
		// 	i->imod[i->imod_count].sfModTransOper = 0;
		// 	i->imod[i->imod_count].sfModDestOper = sfg_initialAttenuation;
		// 	i->imod[i->imod_count].modAmount = 960;
		// 	i->imod[i->imod_count++].sfModAmtSrcOper = 0;
		// }
		i->ibag_count++;

		while (last_value < 0x7F) {
			if (part->samples[y] >= NUM_SAMPLES) {
				//igen[igen_count-2].genAmount.ranges.byHi = 0x7F; //Correct?
				break;
			}

			i->ibag[i->ibag_count].wInstModNdx = i->imod_count;
			i->ibag[i->ibag_count++].wInstGenNdx = i->igen_count;

			i->igen[i->igen_count].sfGenOper = sfg_keyRange;
			i->igen[i->igen_count].genAmount.ranges.byLo = last_value ? last_value + 1 : 0;
			i->igen[i->igen_count++].genAmount.ranges.byHi = part->breaks[y];
			last_value = part->breaks[y];

			i->igen[i->igen_count].sfGenOper = sfg_sampleModes;
			i->igen[i->igen_count++].genAmount.wAmount = sc55->samples[part->samples[y]].loop_mode == 2 ? 0 : 1;

			// igen[igen_count].sfGenOper = 48;
			// igen[igen_count++].genAmount.wAmount = (0x7F - sc55->samples[sc55->parts[x].samples[y]].volume) * 25;

			i->igen[i->igen_count].sfGenOper = sfg_sampleID;
			i->igen[i->igen_count++].genAmount.wAmount = find_or_make_sample(s, sc55->samples, sc55->wave_data, part->samples[y], &params);

			y++;
		}
		return i->inst_count++;
	} else {
		printf("Invalid instrument!!!!\n");
		return 0;
	}
}

int32_t main (int32_t argc, char *argv)
{
	FILE *vf = fopen("build.bin", "rb");
	uint32_t build = 0;
	if (vf) {
		fread(&build, 4, 1, vf);
		fclose(vf);
	}
	build++;

	vf = fopen("build.bin", "wb");
	fwrite(&build, 4, 1, vf);
	fclose(vf);

	struct synth *sc55 = calloc(1, sizeof(struct synth));

	#if defined(MKII)
		FILE *f = fopen("roms"PATH_DIV"SCB-55_R15279828_(program).BIN", "rb");
	#else
		FILE *f = fopen("roms"PATH_DIV"roland_r15209363.ic23", "rb");
	#endif

	//FILE *f = fopen("altered-part-tumpet.rom", "rb");

	fread(sc55->control_data, 0x40000, 1, f);
	fseek(f, 0, SEEK_SET);

	if (!f) {
		printf("Unable to open control rom file.\n");
		return -1;
	}

	// Parse the control ROM into manageable structures
	int32_t inst_count = parse_control_rom(f, sc55->instruments, sc55->parts, sc55->samples, sc55->variations, sc55->drums, banks_sc55, true);

	fclose(f);

	// Decrypt the wave roms
	#if defined(MKII)
		if (!decode_wave_rom_scb(sc55->wave_data)) {
			printf("Unable to process wave ROMS.\n");
			return -1;
		};
	#else
		if (!decode_wave_rom(sc55->wave_data)) {
			printf("Unable to process wave ROMS.\n");
			return -1;
		};
	#endif

	struct ins_partial maxes = {0};
	struct ins_partial mins = {0};

	for (int32_t x = 0; x < sizeof(sc55->instruments[0].parts[1].pp); x++) {
		maxes.pp[x] = mins.pp[x] = sc55->instruments[0].parts[1].pp[x];
	}

	for (int32_t x = 0; x < NUM_INST; x++) {
		if (sc55->instruments[x].name[0] != 0) {
			for (int32_t y = 0; y < sizeof(sc55->instruments[0].parts[1].pp); y++) {
				if (sc55->instruments[x].parts[0].part_index != 0xFFFF) {
					if (sc55->instruments[x].parts[0].pp[y] > maxes.pp[y]) {
						maxes.pp[y] = sc55->instruments[x].parts[0].pp[y];
					};
					if (sc55->instruments[x].parts[0].pp[y] < mins.pp[y]) {
						mins.pp[y] = sc55->instruments[x].parts[0].pp[y];
					};
				}
				if (sc55->instruments[x].parts[1].part_index != 0xFFFF) {
					if (sc55->instruments[x].parts[1].pp[y] > maxes.pp[y]) {
						maxes.pp[y] = sc55->instruments[x].parts[1].pp[y];
					};
					if (sc55->instruments[x].parts[1].pp[y] < mins.pp[y]) {
						mins.pp[y] = sc55->instruments[x].parts[1].pp[y];
					};
				}
			}
		}
	}

	// Produce a soundfont

	struct RIFF *sf2 = NULL;
	char soundfont_name[32] = {0};
	snprintf(soundfont_name, 32, "output"PATH_DIV"AudioFabric-%05d.sf2", build);
	riff_open(&sf2, "sfbk", soundfont_name);

	// Soundfont general parameters LIST
	riff_list_start(sf2, "INFO");

	struct sfVersionTag ver = {0};
	ver.wMajor = 2;
	ver.wMinor = 4;

	char str_buf[256] = {0};
	uint32_t str_len = 0;

	riff_write_chunk(sf2, "ifil", sizeof(struct sfVersionTag), (uint8_t *) &ver);

	str_len = snprintf(str_buf, 256, "%s", "E-mu 10K2");
	riff_write_chunk(sf2, "isng", str_len + ((str_len & 1) ? 1 : 2), str_buf);

	memset(str_buf, 0, sizeof(str_buf));
	str_len = snprintf(str_buf, 256, "%s", "AudioFabric");
	riff_write_chunk(sf2, "INAM", str_len + ((str_len & 1) ? 1 : 2), str_buf);

	memset(str_buf, 0, sizeof(str_buf));
	str_len = snprintf(str_buf, 256, "%s - %s", __DATE__, __TIME__);
	riff_write_chunk(sf2, "ICRD", str_len + ((str_len & 1) ? 1 : 2), str_buf);

	memset(str_buf, 0, sizeof(str_buf));
	str_len = snprintf(str_buf, 256, "%s", "Kitrinx & NewRisingSun");
	riff_write_chunk(sf2, "IENG", str_len + ((str_len & 1) ? 1 : 2), str_buf);

	memset(str_buf, 0, sizeof(str_buf));
	str_len = snprintf(str_buf, 256, "%s %d", "AudioFabric", build);
	riff_write_chunk(sf2, "IPRD", str_len + ((str_len & 1) ? 1 : 2), str_buf);

	memset(str_buf, 0, sizeof(str_buf));
	str_len = snprintf(str_buf, 256, "%s", "SF Builder");
	riff_write_chunk(sf2, "ISFT", str_len + ((str_len & 1) ? 1 : 2), str_buf);

	char comment_buf[UINT16_MAX] = {0};
	char control_ver[11] = {0}; // 0xF380 - 10 v 121, FFF0 - 2.0, ??? - MKII
	char control_date[17] = {0}; // 0xF397 - 16
	char wave_ver[17] = {0}; // 0x20 - 16
	char wave_date[11] = {0}; // 0x30 - 10
	char gs_ver[33] = {0}; // 0x3D148 - 32
	char product_id[17] = {0}; // 3CA08 - 16

	memcpy(control_ver, sc55->control_data + 0xF380, 10);
	memcpy(control_date, sc55->control_data + 0xF397, 16);
	memcpy(wave_ver, sc55->wave_data + 0x20, 16);
	memcpy(wave_date, sc55->wave_data + 0x30, 10);
	memcpy(gs_ver, sc55->control_data + 0x3D148, 32);
	memcpy(product_id, sc55->control_data + 0x3CA08, 16);

	str_len = snprintf(comment_buf, UINT16_MAX, "Number of Presets: %d\nProduct ID: %s\nWave ROM Version: %s\nWave ROM Date: %s\nGS Version: %s\n",
	inst_count, product_id, wave_ver, wave_date, gs_ver);

	riff_write_chunk(sf2, "ICMT", str_len + ((str_len & 1) ? 1 : 2), comment_buf);

	riff_list_end(sf2);


	// Create sample structs to be filled
	struct sf_samples *s = calloc(1, sizeof(struct sf_samples));
	s->sample16 = calloc(600 * 1024 * 1024, sizeof(uint16_t));
	s->sample8 = calloc(600 * 1024 * 1024, sizeof(uint8_t));

	// Process instrument data
	struct sf_instruments *sf_inst = calloc(1, sizeof(struct sf_instruments));
	struct sf_presets *p = calloc(1, sizeof(struct sf_presets));

	struct instrument *ins = NULL;

	for (int32_t y = 0; y < NUM_VARIATIONS; y++) {
		for (int32_t x = 0; x < 128; x++) {
			ins = get_instrument(sc55, x, y);
			if (ins) {
				p->phdr[p->phdr_count].wPreset = x;
				p->phdr[p->phdr_count].wBank = y;
				p->phdr[p->phdr_count].wPresetBagNdx = p->pbag_count;
				p->phdr[p->phdr_count].dwGenre = p->pbag_count;
				p->phdr[p->phdr_count].dwLibrary = p->pbag_count;
				p->phdr[p->phdr_count].dwMorphology = p->pbag_count;

				clean_name(ins->name, p->phdr[p->phdr_count].achPresetName);
				p->pbag[p->pbag_count++].wGenNdx = p->pgen_count;

				p->pgen[p->pgen_count].sfGenOper = sfg_initialAttenuation;
				p->pgen[p->pgen_count++].genAmount.wAmount = PCT2VOL(ins->header[ih_attenuation]);

				if (ins->header[ih_panpot]) {
					p->pgen[p->pgen_count].sfGenOper = sfg_pan;
					p->pgen[p->pgen_count++].genAmount.shAmount = ((double)(ins->header[ih_panpot] - 0x4F) / 79.0) * -500.0;
				}

				p->pbag[p->pbag_count++].wGenNdx = p->pgen_count;


				p->pgen[p->pgen_count].sfGenOper = 41;
				bool used = p->used[sc55->variations[y].variation[x]].used;
				if(used) {
					p->pgen[p->pgen_count++].genAmount.wAmount = p->used[sc55->variations[y].variation[x]].partial0;
				} else {
					p->pgen[p->pgen_count++].genAmount.wAmount = add_instrument(sc55, s, ins->parts[0].part_index, &ins->parts[0], sf_inst, ins);
					p->used[sc55->variations[y].variation[x]].used = true;
					p->used[sc55->variations[y].variation[x]].partial0 = p->pgen[p->pgen_count - 1].genAmount.wAmount;
				}

				if (ins->parts[1].part_index < NUM_PARTS) {
					p->pbag[p->pbag_count++].wGenNdx = p->pgen_count;

					p->pgen[p->pgen_count].sfGenOper = 41;
					if(used) {
						p->pgen[p->pgen_count++].genAmount.wAmount = p->used[sc55->variations[y].variation[x]].partial1;
					} else {
						p->pgen[p->pgen_count++].genAmount.wAmount = add_instrument(sc55, s, ins->parts[0].part_index, &ins->parts[1], sf_inst, ins);
						p->used[sc55->variations[y].variation[x]].partial1 = p->pgen[p->pgen_count - 1].genAmount.wAmount;
					}
				}
				p->phdr_count++;
			}
		}
	}

	// Deal with goofy drum bank conversion
	uint32_t drum_start = sf_inst->inst_count;
	for (int32_t x = 0; x < NUM_DRUMS; x++) {
		clean_name(sc55->drums[x].name, sf_inst->inst[sf_inst->inst_count].achInstName);
		printf("Adding %s at %d\n", sf_inst->inst[sf_inst->inst_count].achInstName, sf_inst->inst_count);
		sf_inst->inst[sf_inst->inst_count++].wInstBagNdx = sf_inst->ibag_count;

		for (int32_t y = 27; y < 108; y++) {
			if (sc55->drums[x].preset[y] < NUM_INST) {
				for (int32_t z = 0; z < 2; z++) {

					if (sc55->instruments[sc55->drums[x].preset[y]].parts[z].part_index >= NUM_PARTS) continue;
					struct part *part = &sc55->parts[sc55->instruments[sc55->drums[x].preset[y]].parts[z].part_index];

					if (part->samples[0] > NUM_SAMPLES) continue;

					sf_inst->ibag[sf_inst->ibag_count].wInstModNdx = sf_inst->imod_count;
					sf_inst->ibag[sf_inst->ibag_count++].wInstGenNdx = sf_inst->igen_count;

					sf_inst->igen[sf_inst->igen_count].sfGenOper = 43;
					sf_inst->igen[sf_inst->igen_count].genAmount.ranges.byLo = y;
					sf_inst->igen[sf_inst->igen_count++].genAmount.ranges.byHi = y;

					//add_igen_word(sf_inst, sfg_keynum, sc55->drums[x].key[y]);

					uint16_t new_key = y - (sc55->drums[x].key[y] - sc55->samples[part->samples[0]].root_key);
					sf_inst->igen[sf_inst->igen_count].sfGenOper = sfg_overridingRootKey;
					sf_inst->igen[sf_inst->igen_count++].genAmount.shAmount = new_key;


					sf_inst->igen[sf_inst->igen_count].sfGenOper = sfg_initialAttenuation;
					sf_inst->igen[sf_inst->igen_count++].genAmount.shAmount = PCT2VOL(sc55->drums[x].volume[y]);

					double ih_pan = ((double)((sc55->instruments[sc55->drums[x].preset[y]].header[ih_panpot] ? sc55->instruments[sc55->drums[x].preset[y]].header[ih_panpot] : 0x4F) - 0x4F) / 79.0) * 500.0;
					double drum_pan =((double)((sc55->drums[x].panpot[y] ? sc55->drums[x].panpot[y] : 0x40) - 0x40) / 64.0);

					drum_pan = ih_panpot + drum_pan;
					if (drum_pan > 1.0)
						drum_pan = 1.0;
					else if (drum_pan < -1.0)
						drum_pan = -1.0;

					sf_inst->igen[sf_inst->igen_count].sfGenOper = sfg_exclusiveClass;
					sf_inst->igen[sf_inst->igen_count++].genAmount.shAmount = sc55->drums[x].assignGroup[y];

					struct instrument *preset = &sc55->instruments[sc55->drums[x].preset[y]];
					struct sample_params params = {0};
					add_instrument_params(&sc55->instruments[sc55->drums[x].preset[y]].parts[z],
						sf_inst, &sc55->instruments[sc55->drums[x].preset[y]], true, &sc55->drums[x],
						y, sc55, &params);

					sf_inst->igen[sf_inst->igen_count].sfGenOper = sfg_sampleModes;
					sf_inst->igen[sf_inst->igen_count++].genAmount.wAmount = sc55->samples[part->samples[0]].loop_mode == 0;

					if ( part->samples[0] > NUM_SAMPLES) {
						printf("out of range sample %04X for drum %d\n", part->samples[0], y);
					}
					sf_inst->igen[sf_inst->igen_count].sfGenOper = sfg_sampleID;
					sf_inst->igen[sf_inst->igen_count++].genAmount.wAmount = find_or_make_sample(s, sc55->samples, sc55->wave_data, part->samples[0], &params);
				}
			}
		}
	}

	uint8_t drum_banks[NUM_DRUMS] = {0, 8, 16, 24, 25, 32, 40, 48, 56, 57, 58, 59, 60, 127};
	for (int32_t x = 0; x < NUM_DRUMS; x++) {
		p->phdr[p->phdr_count].wPreset = drum_banks[x];
		p->phdr[p->phdr_count].wBank = 128;
		p->phdr[p->phdr_count].wPresetBagNdx = p->pbag_count;
		p->phdr[p->phdr_count].dwGenre = p->pbag_count;
		p->phdr[p->phdr_count].dwLibrary = p->pbag_count;
		p->phdr[p->phdr_count].dwMorphology = p->pbag_count;

		clean_name(sc55->drums[x].name, p->phdr[p->phdr_count].achPresetName);
		printf("Adding bag for %s %d\n", p->phdr[p->phdr_count].achPresetName, drum_start + x);
		p->pbag[p->pbag_count++].wGenNdx = p->pgen_count;

		p->pgen[p->pgen_count].sfGenOper = 41;
		p->pgen[p->pgen_count++].genAmount.wAmount = drum_start + x;

		p->phdr_count++;
	}

	// Set up terminal elements and calculate final sizes per spec.

	// The terminal headers must point to terminal bags, and the terminal bags
	// must point to terminal gens and mods. These records are used to determine
	// the size of the preceeding entry and the soundfont will not work correctly
	// if these aren't set properly.

	snprintf(sf_inst->inst[sf_inst->inst_count].achInstName, 20, "%s", "EOI");
	sf_inst->inst[sf_inst->inst_count].wInstBagNdx = sf_inst->ibag_count;

	snprintf(p->phdr[p->phdr_count].achPresetName, 20, "%s", "EOP");
	p->phdr[p->phdr_count].wPresetBagNdx = p->pbag_count;

	snprintf(s->shdr[s->num_samples].achSampleName, 20, "%s", "EOS");

	sf_inst->ibag[sf_inst->ibag_count].wInstGenNdx = sf_inst->igen_count;
	sf_inst->ibag[sf_inst->ibag_count].wInstModNdx = sf_inst->imod_count;
	p->pbag[p->pbag_count].wGenNdx = p->pgen_count;
	p->pbag[p->pbag_count].wModNdx = p->pmod_count;

	// This may seem like a crappy way to calculate sizes, but the spec is very specific and it's
	// easy for this to get out of whack, it makes it more robust.

	uint32_t ibag_size = (sf_inst->inst[sf_inst->inst_count].wInstBagNdx * 4) + 4;
	uint32_t pbag_size = (p->phdr[p->phdr_count].wPresetBagNdx * 4) + 4;
	uint32_t igen_size = (sf_inst->ibag[sf_inst->inst[sf_inst->inst_count].wInstBagNdx].wInstGenNdx * 4) + 4;
	uint32_t pgen_size = (p->pbag[p->phdr[p->phdr_count].wPresetBagNdx].wGenNdx * 4) + 4;
	uint32_t imod_size = (sf_inst->ibag[sf_inst->inst[sf_inst->inst_count].wInstBagNdx].wInstModNdx * 10) + 10;
	uint32_t pmod_size = (p->pbag[p->phdr[p->phdr_count].wPresetBagNdx].wModNdx * 10) + 10;


	// SoundFont Sample Data LIST
	riff_list_start(sf2, "sdta");

	riff_write_chunk(sf2, "smpl", (s->data_size) * 2, (uint8_t *) s->sample16);
	riff_write_chunk(sf2, "sm24", (s->data_size & 1) ? s->data_size + 1 : s->data_size, s->sample8);

	printf("Total number of samples: %d data size: %ld bytes\n", s->num_samples, s->data_size * 2 + s->data_size);
	free(s->sample16);
	free(s->sample8);


	riff_list_end(sf2);

	// Write "Hydra" LIST
	riff_list_start(sf2, "pdta");

	riff_write_chunk(sf2, "phdr", (p->phdr_count + 1) * sizeof(struct sfPresetHeader), (uint8_t *) p->phdr);
	riff_write_chunk(sf2, "pbag", pbag_size, (uint8_t *) p->pbag);
	riff_write_chunk(sf2, "pmod", pmod_size, (uint8_t *) p->pmod);
	riff_write_chunk(sf2, "pgen", pgen_size, (uint8_t *) p->pgen);

	riff_write_chunk(sf2, "inst", (sf_inst->inst_count + 1) * sizeof(struct sfInst), (uint8_t *)sf_inst->inst);
	riff_write_chunk(sf2, "ibag", ibag_size, (uint8_t *) sf_inst->ibag);
	riff_write_chunk(sf2, "imod", imod_size, (uint8_t *) sf_inst->imod);
	riff_write_chunk(sf2, "igen", igen_size, (uint8_t *) sf_inst->igen);

	riff_write_chunk(sf2, "shdr", (s->num_samples + 1) * sizeof(struct sfSample), (uint8_t *) s->shdr);
	riff_list_end(sf2);

	riff_close(&sf2);

	free(s);
	free(p);
	free(sf_inst);
	free(sc55);

	return 0;
}