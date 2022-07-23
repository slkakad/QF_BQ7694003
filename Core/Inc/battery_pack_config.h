#ifndef BATTERY_PACK_CONFIG_H
#define BATTERY_PACK_CONFIG_H
#endif

typedef struct
{
	float bat_maxV;
	float bat_minV;
  float bat_maxC;
	float bat_minC;
	float max_soc;
	float min_soc;
	float max_temp;
	float min_temp;
	float SoC;
	float SoH;
}battery_config;

typedef struct
{
	float cell_OV; 
	float cell_UV;
	float cell_OT; 
	float cell_UT;
}cell_config;
