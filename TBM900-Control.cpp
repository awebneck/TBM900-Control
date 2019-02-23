#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include <string.h>
#include <thread>
#include <mutex>
#include <queue>
#include <map>
#include <vector>
#include "rs232.h"
#include <iostream>
#include <cstring>
#include <chrono>
#include "ip/UdpSocket.h"
#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "osc/OscOutboundPacketStream.h"

#if IBM
#include <windows.h>
#endif
#if LIN
#include <GL/gl.h>
#elif __GNUC__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#ifndef OSC_BUNDLE_SIZE
#define OSC_BUNDLE_SIZE 4092
#endif
#ifndef XPLM300
#error This is made to be compiled against the XPLM300 SDK
#endif
#ifndef CPORT_NR
#define CPORT_NR 6
#endif
#ifndef BDRATE
#define BDRATE 9600
#endif
#ifndef MSG_ADD_DATAREF
#define MSG_ADD_DATAREF 0x01000000
#endif
#ifndef EMERGENCY_OVERRIDE_AXIS
#define EMERGENCY_OVERRIDE_AXIS 12
#endif
#ifndef SMALL_THROTTLE_AXIS
#define SMALL_THROTTLE_AXIS 10
#endif

struct rotenc_t {
	XPLMCommandRef *upCommand;
	XPLMCommandRef *downCommand;
	int period;
	int eventCount;
};

struct button_t {
	XPLMCommandRef *actuateCommand;
	bool held;
};

struct toggle_switch_t {
	XPLMCommandRef *onCommand;
	XPLMCommandRef *offCommand;
};

struct triple_switch_t {
	XPLMCommandRef *upCommand;
	XPLMCommandRef *midCommand;
	XPLMCommandRef *downCommand;
};
struct direct_writable_t {
	XPLMDataRef *dataRef;
	int len;
	int index;
	int dRefSize;
};

struct direct_write_t {
	XPLMDataRef *dataRef;
	float value;
	int len;
	int index;
	int dRefSize;
};

struct hold_t {
	XPLMCommandRef *actuateCommand;
	bool on;
};

static XPLMCommandRef baroUpCommand,
baroDownCommand,
baroStdCommand,
crashBarUpCommand,
crashBarDownCommand,
srcGpuCommand,
srcBattCommand,
srcOffCommand,
genResetMainCommand,
genResetStbyCommand,
genStbyCommand,
genMainCommand,
genOffCommand,
extLtsLdgCommand,
extLtsTaxiCommand,
extLtsOffCommand,
ltsPulseOnCommand,
ltsPulseOffCommand,
ltsNavOnCommand,
ltsNavOffCommand,
ltsStrobeOnCommand,
ltsStrobeOffCommand,
ltsDimmerOnCommand,
ltsDimmerOffCommand,
ltsCabinOnCommand,
ltsCabinOffCommand,
ltsAccessCommand,
starterOnCommand,
starterAbortCommand,
ignitionAutoCommand,
ignitionOnCommand,
ignitionOffCommand,
auxBpAutoCommand,
auxBpOnCommand,
auxBpOffCommand,
fuelSelAutoCommand,
fuelSelManualCommand,
fuelSelShiftCommand,
apTrimsOnCommand,
apTrimsApOffCommand,
apTrimsOffCommand,
eltOnCommand,
eltArmCommand,
eltTestCommand,
airframeDeiceOnCommand,
airframeDeiceOffCommand,
iceLightOnCommand,
iceLightOffCommand,
propDeiceOnCommand,
propDeiceOffCommand,
windShieldOnCommand,
windShieldOffCommand,
pitotLOnCommand,
pitotLOffCommand,
pitotROnCommand,
pitotROffCommand,
inertSepOnCommand,
inertSepOffCommand,
iceLightsTestCommand,
gearUpCommand,
gearDownCommand,
gearLightsTestCommand,
gearCheckCommand,
acAutoCommand,
acManualCommand,
acOffCommand,
bleedAutoCommand,
bleedOffCommand,
presModeAutoCommand,
presModeMaxDiffCommand,
clearMasterWarningCommand,
clearMasterCautionCommand,
setFuelLeftCommand,
setFuelRightCommand,
setFuelNoneCommand,
altStaticToggleCommand,
ramAirToggleCommand,
gpuToggleCommand,
chocksToggleCommand,
pilotDoorToggleCommand,
pilotDoorLockToggleCommand,
mainDoorToggleCommand,
mainDoorLockToggleCommand,
apHdgDirUpCommand,
apHdgDirDownCommand,
apHdgDirSyncCommand,
apHdgCommand,
apAprCommand,
apBcCommand,
apNavCommand,
apFdCommand,
apBankCommand,
apCrs1UpCommand,
apCrs1DownCommand,
apCrs1DirCommand,
apApCommand,
apYdCommand,
apXfrCommand,
apAltCommand,
apAltCoarseUpCommand,
apAltCoarseDownCommand,
apAltFineUpCommand,
apAltFineDownCommand,
apVsCommand,
apNoseUpCommand,
apNoseDownCommand,
apVnavCommand,
apFlcCommand,
apSpdCommand,
apCrs2UpCommand,
apCrs2DownCommand,
apCrs2DirCommand,
radNavSwapCommand,
radNav12Command,
radNavCoarseUpCommand,
radNavCoarseDownCommand,
radNavFineUpCommand,
radNavFineDownCommand,
radNavVolumeUpCommand,
radNavVolumeDownCommand,
radComSwapCommand,
radCom12Command,
radComCoarseUpCommand,
radComCoarseDownCommand,
radComFineUpCommand,
radComFineDownCommand,
radComVolumeUpCommand,
radComVolumeDownCommand,
audioMicCom1Command,
audioMonCom1Command,
audioMicCom2Command,
audioMonCom2Command,
audioMonDmeCommand,
audioMonAdfCommand,
audioMonNav1Command,
audioMonNav2Command,
microMaskCoverToggleCommand,
microMaskToggleCommand,
fmsPfdSK1Command,
fmsPfdSK2Command,
fmsPfdSK3Command,
fmsPfdSK4Command,
fmsPfdSK5Command,
fmsPfdSK6Command,
fmsPfdSK7Command,
fmsPfdSK8Command,
fmsPfdSK9Command,
fmsPfdSK10Command,
fmsPfdSK11Command,
fmsPfdSK12Command,
fmsPfdFmsCoarseUpCommand,
fmsPfdFmsCoarseDownCommand,
fmsPfdFmsFineUpCommand,
fmsPfdFmsFineDownCommand,
fmsPfdFmsCursorCommand,
fmsPfdFmsDirCommand,
fmsPfdFmsMenuCommand,
fmsPfdFmsFplCommand,
fmsPfdFmsProcCommand,
fmsPfdFmsClrCommand,
fmsPfdFmsEntCommand,
fmsMfdSK1Command,
fmsMfdSK2Command,
fmsMfdSK3Command,
fmsMfdSK4Command,
fmsMfdSK5Command,
fmsMfdSK6Command,
fmsMfdSK7Command,
fmsMfdSK8Command,
fmsMfdSK9Command,
fmsMfdSK10Command,
fmsMfdSK11Command,
fmsMfdSK12Command,
fmsMfdFmsCoarseUpCommand,
fmsMfdFmsCoarseDownCommand,
fmsMfdFmsFineUpCommand,
fmsMfdFmsFineDownCommand,
fmsMfdFmsCursorCommand,
fmsMfdFmsDirCommand,
fmsMfdFmsMenuCommand,
fmsMfdFmsFplCommand,
fmsMfdFmsProcCommand,
fmsMfdFmsClrCommand,
fmsMfdFmsEntCommand,
fmsMfdFmsBkspCommand,
fmsMfdFmsSpcCommand,
fmsMfdRangeUpCommand,
fmsMfdRangeDownCommand,
fmsMfdPanCommand,
fmsMfdPanUpCommand,
fmsMfdPanDownCommand,
fmsMfdPanLeftCommand,
fmsMfdPanRightCommand,
fmsMfdPanUpLeftCommand,
fmsMfdPanUpRightCommand,
fmsMfdPanDownLeftCommand,
fmsMfdPanDownRightCommand,
stbyMCommand,
stbySCommand,
stbyMinusCommand,
stbyPlusCommand,
fmsMfdKpA,
fmsMfdKpB,
fmsMfdKpC,
fmsMfdKpD,
fmsMfdKpE,
fmsMfdKpF,
fmsMfdKpG,
fmsMfdKpH,
fmsMfdKpI,
fmsMfdKpJ,
fmsMfdKpK,
fmsMfdKpL,
fmsMfdKpM,
fmsMfdKpN,
fmsMfdKpO,
fmsMfdKpP,
fmsMfdKpQ,
fmsMfdKpR,
fmsMfdKpS,
fmsMfdKpT,
fmsMfdKpU,
fmsMfdKpV,
fmsMfdKpW,
fmsMfdKpX,
fmsMfdKpY,
fmsMfdKpZ,
fmsMfdKp1,
fmsMfdKp2,
fmsMfdKp3,
fmsMfdKp4,
fmsMfdKp5,
fmsMfdKp6,
fmsMfdKp7,
fmsMfdKp8,
fmsMfdKp9,
fmsMfdKp0,
fmsMfdKpPm,
fmsMfdKpDot,
flapsUpCommand,
flapsTakeoffCommand,
flapsLandingCommand,
mixtureDownCommand,
mixtureUpCommand;

static XPLMDataRef baroSettingDR,
crashBarDR,
srcDR,
genDR,
ltsExtDR,
ltsPulseDR,
ltsNavDR,
ltsStrobeDR,
ltsPanelDR,
ltsDimmerDR,
ltsCabinDR,
ignitionDR,
auxBpDR,
fuelSelDR,
apTrimsDR,
eltDR,
airframeDeiceDR,
airframeDeiceLightsDR,
iceLightDR,
propDeiceDR,
propDeiceLightDR,
windShieldDR,
windShieldLightsDR,
pitotLDR,
pitotRDR,
inertSepDR,
parkingBrakeDR,
acDR,
bleedDR,
presModeDR,
dumpGuardDR,
dumpStateDR,
dumpDR,
fanSpeedDR,
tempZoneDR,
tempDR,
hotAirFlowDR,
manFuelSelDR,
masterCautionDR,
masterWarningDR,
gearHandleDR,
gearSafeLightDR,
gearUnsafeLightDR,
altStaticDR,
ramAirDR,
pilotDoorDR,
pilotDoorLockDR,
mainDoorDR,
mainDoorLockDR,
gpuDR,
tiesDR,
wickCoverDR,
fuelCapDR,
pitotCoverDR,
chocksDR,
fuelQtyLDR,
fuelQtyRDR,
staticCoverDR,
readTimeDR,
writeTimeDR,
apHdgDirDR,
apCrs1DR,
apAltDR,
apVsDR,
apFlcDR,
apCrs2DR,
audioMicCom1DR,
audioMonCom1DR,
audioMicCom2DR,
audioMonCom2DR,
audioMonDmeDR,
audioMonAdfDR,
audioMonNav1DR,
audioMonNav2DR,
radCom1FreqActiveDR,
radCom1FreqStbyDR,
radCom2FreqActiveDR,
radCom2FreqStbyDR,
radNav1FreqActiveDR,
radNav1FreqStbyDR,
radNav2FreqActiveDR,
radNav2FreqStbyDR,
altDR,
kiasDR,
vsDR,
gsDR,
apfdDR,
apHdgIndDR,
apAprIndDR,
apBcIndDR,
apNavIndDR,
apBankIndDR,
apXfrLIndDR,
apXfrRIndDR,
apApIndDR,
apYdIndDR,
apAltIndDR,
apVsIndDR,
apVnavIndDR,
apFlcIndDR,
flaprqstDR,
emergPowerDR,
joystickAxisValuesDR;

static std::map<std::string, rotenc_t> rotencsByOscRef{
	{"/radio_ap/baro", {&baroUpCommand, &baroDownCommand, 5, 0} },
	{"/radio_ap/hdg_sel", {&apHdgDirUpCommand, &apHdgDirDownCommand, 5, 0} },
	{"/radio_ap/crs1", {&apCrs1UpCommand, &apCrs1DownCommand, 5, 0} },
	{"/radio_ap/alt_sel_coarse", {&apAltCoarseUpCommand, &apAltCoarseDownCommand, 5, 0} },
	{"/radio_ap/alt_sel_fine", {&apAltFineUpCommand, &apAltFineDownCommand, 5, 0} },
	{"/radio_ap/crs2", {&apCrs2UpCommand, &apCrs2DownCommand, 5, 0} },
	{"/radio_ap/nav_coarse", {&radNavCoarseUpCommand, &radNavCoarseDownCommand, 5, 0} },
	{"/radio_ap/nav_fine", {&radNavFineUpCommand, &radNavFineDownCommand, 5, 0} },
	{"/radio_ap/nav_vol", {&radNavVolumeUpCommand, &radNavVolumeDownCommand, 5, 0} },
	{"/radio_ap/com_coarse", {&radComCoarseUpCommand, &radComCoarseDownCommand, 5, 0} },
	{"/radio_ap/com_fine", {&radComFineUpCommand, &radComFineDownCommand, 5, 0} },
	{"/radio_ap/com_vol", {&radComVolumeUpCommand, &radComVolumeDownCommand, 5, 0} },
	{"/fms/pfd_fms_coarse", {&fmsPfdFmsCoarseUpCommand, &fmsPfdFmsCoarseDownCommand, 5, 0} },
	{"/fms/pfd_fms_fine", {&fmsPfdFmsFineUpCommand, &fmsPfdFmsFineDownCommand, 5, 0} },
	{"/fms/mfd_fms_coarse", {&fmsMfdFmsCoarseUpCommand, &fmsMfdFmsCoarseDownCommand, 5, 0} },
	{"/fms/mfd_fms_fine", {&fmsMfdFmsFineUpCommand, &fmsMfdFmsFineDownCommand, 5, 0} },
	{"/fms/range", {&fmsMfdRangeUpCommand, &fmsMfdRangeDownCommand, 5, 0} },
};

static std::map<std::string, button_t> buttonsByOscRef{
	{"/util/power_src/3/1", {&srcGpuCommand, false} },
	{"/util/power_src/2/1", {&srcBattCommand, false} },
	{"/util/power_src/1/1", {&srcOffCommand, false} },
	{"/util/power_gen/3/1", {&genStbyCommand, false} },
	{"/util/power_gen/2/1", {&genMainCommand, false} },
	{"/util/power_gen/1/1", {&genOffCommand, false} },
	{"/util/gen_reset_main", {&genResetMainCommand, true} },
	{"/util/gen_reset_stby", {&genResetStbyCommand, true} },
	{"/util/lts_ext/3/1", {&extLtsLdgCommand, false} },
	{"/util/lts_ext/2/1", {&extLtsTaxiCommand, false} },
	{"/util/lts_ext/1/1", {&extLtsOffCommand, false} },
	{"/util/lts_pulse/2/1", {&ltsPulseOnCommand, false} },
	{"/util/lts_pulse/1/1", {&ltsPulseOffCommand, false} },
	{"/util/lts_nav/2/1", {&ltsNavOnCommand, false} },
	{"/util/lts_nav/1/1", {&ltsNavOffCommand, false} },
	{"/util/lts_strobe/2/1", {&ltsStrobeOnCommand, false} },
	{"/util/lts_strobe/1/1", {&ltsStrobeOffCommand, false} },
	{"/util/lts_dimmer/2/1", {&ltsDimmerOnCommand, false} },
	{"/util/lts_dimmer/1/1", {&ltsDimmerOffCommand, false} },
	{"/util/lts_cabin/2/1", {&ltsCabinOnCommand, false} },
	{"/util/lts_cabin/1/1", {&ltsCabinOffCommand, false} },
	{"/util/lts_access", {&ltsAccessCommand, true} },
	{"/util/starter/3/1", {&starterOnCommand, true} },
	{"/util/starter/1/1", {&starterAbortCommand, true} },
	{"/util/ignition/3/1", {&ignitionAutoCommand, false} },
	{"/util/ignition/2/1", {&ignitionOnCommand, false} },
	{"/util/ignition/1/1", {&ignitionOffCommand, false} },
	{"/util/aux_bp/3/1", {&auxBpAutoCommand, false} },
	{"/util/aux_bp/2/1", {&auxBpOnCommand, false} },
	{"/util/aux_bp/1/1", {&auxBpOffCommand, false} },
	{"/util/fuel_sel/2/1", {&fuelSelAutoCommand, false} },
	{"/util/fuel_sel/1/1", {&fuelSelManualCommand, false} },
	{"/util/fuel_sel_shift", {&fuelSelShiftCommand, true} },
	{"/util/ap_trims/3/1", {&apTrimsOnCommand, false} },
	{"/util/ap_trims/2/1", {&apTrimsApOffCommand, false} },
	{"/util/ap_trims/1/1", {&apTrimsOffCommand, false} },
	{"/util/elt/3/1", {&eltOnCommand, false} },
	{"/util/elt/2/1", {&eltArmCommand, false} },
	{"/util/elt/1/1", {&eltTestCommand, false} },
	{"/util/airframe_deice/2/1", {&airframeDeiceOnCommand, false} },
	{"/util/airframe_deice/1/1", {&airframeDeiceOffCommand, false} },
	{"/util/ice_light/2/1", {&iceLightOnCommand, false} },
	{"/util/ice_light/1/1", {&iceLightOffCommand, false} },
	{"/util/prop_deice/2/1", {&propDeiceOnCommand, false} },
	{"/util/prop_deice/1/1", {&propDeiceOffCommand, false} },
	{"/util/wind_shield/2/1", {&windShieldOnCommand, false} },
	{"/util/wind_shield/1/1", {&windShieldOffCommand, false} },
	{"/util/pitot_l/2/1", {&pitotLOnCommand, false} },
	{"/util/pitot_l/1/1", {&pitotLOffCommand, false} },
	{"/util/pitot_r/2/1", {&pitotROnCommand, false} },
	{"/util/pitot_r/1/1", {&pitotROffCommand, false} },
	{"/util/inert_sep/2/1", {&inertSepOnCommand, false} },
	{"/util/inert_sep/1/1", {&inertSepOffCommand, false} },
	{"/util/ice_lts_test", {&iceLightsTestCommand, true} },
	{"/util/man_fuel_sel/1/1", {&setFuelLeftCommand, false} },
	{"/util/man_fuel_sel/1/3", {&setFuelRightCommand, false} },
	{"/util/man_fuel_sel/1/2", {&setFuelNoneCommand, false} },
	
	{ "/common/mw_clear", {&clearMasterWarningCommand, true } },
	{ "/common/mc_clear", {&clearMasterCautionCommand, true } },
	
	{ "/util/gear/2/1", {&gearUpCommand, false} },
	{ "/util/gear/1/1", {&gearDownCommand, false} },
	{ "/util/gear_lts_test", {&gearLightsTestCommand, true} },
	{ "/util/gear_check", {&gearCheckCommand, true} },
	{ "/util/ac/3/1", {&acAutoCommand, false} },
	{ "/util/ac/2/1", {&acManualCommand, false} },
	{ "/util/ac/1/1", {&acOffCommand, false} },
	{ "/util/bleed/2/1", {&bleedAutoCommand, false} },
	{ "/util/bleed/1/1", {&bleedOffCommand, false} },
	{ "/util/pres_mode/2/1", {&presModeAutoCommand, false} },
	{ "/util/pres_mode/1/1", {&presModeMaxDiffCommand, false} },
	{ "/util/ram_air_act", {&ramAirToggleCommand, true} },
	
	{ "/ext/main_door_act", {&mainDoorToggleCommand, true} },
	
	{ "/radio_ap/hdg_sel_step_up", {&apHdgDirUpCommand, false} },
	{ "/radio_ap/hdg_sel_step_dn", {&apHdgDirDownCommand, false} },
	{ "/radio_ap/hdg_sync", {&apHdgDirSyncCommand, false} },
	{ "/radio_ap/hdg", {&apHdgCommand, false} },
	{ "/radio_ap/apr", {&apAprCommand, false} },
	{ "/radio_ap/bc", {&apBcCommand, false} },
	{ "/radio_ap/nav", {&apNavCommand, false} },
	{ "/radio_ap/crs1_step_up", {&apCrs1UpCommand, false} },
	{ "/radio_ap/crs2_step_dn", {&apCrs1DownCommand, false} },
	{ "/radio_ap/crs1_dir", {&apCrs2DirCommand, false} },
	{ "/radio_ap/fd", {&apFdCommand, false} },
	{ "/radio_ap/bank", {&apBankCommand, false} },
	{ "/radio_ap/ap", {&apApCommand, false} },
	{ "/radio_ap/xfr", {&apXfrCommand, false} },
	{ "/radio_ap/yd", {&apYdCommand, false} },
	{ "/radio_ap/alt_sel_coarse_step_up", {&apAltCoarseUpCommand, false} },
	{ "/radio_ap/alt_sel_coarse_step_dn", {&apAltCoarseDownCommand, false} },
	{ "/radio_ap/alt_sel_fine_step_up", {&apAltFineUpCommand, false} },
	{ "/radio_ap/alt_sel_fine_step_dn", {&apAltFineDownCommand, false} },
	{ "/radio_ap/alt", {&apAltCommand, false} },
	{ "/radio_ap/vs", {&apVsCommand, false} },
	{ "/radio_ap/vnav", {&apVnavCommand, false} },
	// FIXME: Need Manual Hold
	{ "/radio_ap/vs_sel/2/1", {&apNoseDownCommand, true} },
	// FIXME: Need Manual Hold
	{ "/radio_ap/vs_sel/1/1", {&apNoseUpCommand, true} },
	{ "/radio_ap/flc", {&apFlcCommand, false} },
	{ "/radio_ap/spd", {&apSpdCommand, false} },
	{ "/radio_ap/crs2_dir", {&apCrs2DirCommand, false} },

	{ "/radio_ap/baro_step_up", {&baroUpCommand, false} },
	{ "/radio_ap/baro_step_dn", {&baroDownCommand, false} },
	{ "/radio_ap/baro_std", {&baroStdCommand, false} },

	{ "/radio_ap/nav_swap", {&radNavSwapCommand, true} },
	{ "/radio_ap/nav_coarse_step_up", {&radNavCoarseUpCommand, false} },
	{ "/radio_ap/nav_coarse_step_dn", {&radNavCoarseDownCommand, false} },
	{ "/radio_ap/nav_fine_step_up", {&radNavFineUpCommand, false} },
	{ "/radio_ap/nav_fine_step_dn", {&radNavFineDownCommand, false} },
	{ "/radio_ap/nav_12", {&radNav12Command, false} },
	{ "/radio_ap/com_swap", {&radComSwapCommand, true} },
	{ "/radio_ap/com_coarse_step_up", {&radComCoarseUpCommand, false} },
	{ "/radio_ap/com_coarse_step_dn", {&radComCoarseDownCommand, false} },
	{ "/radio_ap/com_fine_step_up", {&radComFineUpCommand, false} },
	{ "/radio_ap/com_fine_step_dn", {&radComFineDownCommand, false} },
	{ "/radio_ap/com_12", {&radCom12Command, false} },
	{ "/radio_ap/com1_mic", {&audioMicCom1Command, false} },
	{ "/radio_ap/com1_mon", {&audioMonCom1Command, false} },
	{ "/radio_ap/com2_mic", {&audioMicCom2Command, false} },
	{ "/radio_ap/com2_mon", {&audioMonCom2Command, false} },
	{ "/radio_ap/dme_mon", {&audioMonDmeCommand, false} },
	{ "/radio_ap/adf_mon", {&audioMonAdfCommand, false} },
	{ "/radio_ap/nav1_mon", {&audioMonNav1Command, false} },
	{ "/radio_ap/nav2_mon", {&audioMonNav2Command, false} },
	{ "/radio_ap/stby_m", {&stbyMCommand, true} },
	{ "/radio_ap/stby_s", {&stbySCommand, true} },
	{ "/radio_ap/stby_minus", {&stbyMinusCommand, true} },
	{ "/radio_ap/stby_plus", {&stbyPlusCommand, true} },
	// FIXME: Implement
	//{ "/radio_ap/micro_mask", {&mainDoorToggleCommand, false} },

	{ "/fms/pfd_sk_1", {&fmsPfdSK1Command, true} },
	{ "/fms/pfd_sk_2", {&fmsPfdSK2Command, true} },
	{ "/fms/pfd_sk_3", {&fmsPfdSK3Command, true} },
	{ "/fms/pfd_sk_4", {&fmsPfdSK4Command, true} },
	{ "/fms/pfd_sk_5", {&fmsPfdSK5Command, true} },
	{ "/fms/pfd_sk_6", {&fmsPfdSK6Command, true} },
	{ "/fms/pfd_sk_7", {&fmsPfdSK7Command, true} },
	{ "/fms/pfd_sk_8", {&fmsPfdSK8Command, true} },
	{ "/fms/pfd_sk_9", {&fmsPfdSK9Command, true} },
	{ "/fms/pfd_sk_10", {&fmsPfdSK10Command, true} },
	{ "/fms/pfd_sk_11", {&fmsPfdSK11Command, true} },
	{ "/fms/pfd_sk_12", {&fmsPfdSK12Command, true} },

	{ "/fms/mfd_sk_1", {&fmsMfdSK1Command, true} },
	{ "/fms/mfd_sk_2", {&fmsMfdSK2Command, true} },
	{ "/fms/mfd_sk_3", {&fmsMfdSK3Command, true} },
	{ "/fms/mfd_sk_4", {&fmsMfdSK4Command, true} },
	{ "/fms/mfd_sk_5", {&fmsMfdSK5Command, true} },
	{ "/fms/mfd_sk_6", {&fmsMfdSK6Command, true} },
	{ "/fms/mfd_sk_7", {&fmsMfdSK7Command, true} },
	{ "/fms/mfd_sk_8", {&fmsMfdSK8Command, true} },
	{ "/fms/mfd_sk_9", {&fmsMfdSK9Command, true} },
	{ "/fms/mfd_sk_10", {&fmsMfdSK10Command, true} },
	{ "/fms/mfd_sk_11", {&fmsMfdSK11Command, true} },
	{ "/fms/mfd_sk_12", {&fmsMfdSK12Command, true} },

	{ "/fms/pfd_fms_coarse_step_up", {&fmsPfdFmsCoarseUpCommand, false} },
	{ "/fms/pfd_fms_coarse_step_dn", {&fmsPfdFmsCoarseDownCommand, false} },
	{ "/fms/pfd_fms_fine_step_up", {&fmsPfdFmsFineUpCommand, false} },
	{ "/fms/pfd_fms_fine_step_dn", {&fmsPfdFmsFineDownCommand, false} },
	{ "/fms/pfd_fms_crsr", {&fmsPfdFmsCursorCommand, true} },
	{ "/fms/pfd_dir", {&fmsPfdFmsDirCommand, true} },
	{ "/fms/pfd_menu", {&fmsPfdFmsMenuCommand, true} },
	{ "/fms/pfd_fpl", {&fmsPfdFmsFplCommand, true} },
	{ "/fms/pfd_proc", {&fmsPfdFmsProcCommand, true} },
	{ "/fms/pfd_clr", {&fmsPfdFmsClrCommand, true} },
	{ "/fms/pfd_ent", {&fmsPfdFmsEntCommand, true} },

	{ "/fms/mfd_fms_coarse_step_up", {&fmsMfdFmsCoarseUpCommand, false} },
	{ "/fms/mfd_fms_coarse_step_dn", {&fmsMfdFmsCoarseDownCommand, false} },
	{ "/fms/mfd_fms_fine_step_up", {&fmsMfdFmsFineUpCommand, false} },
	{ "/fms/mfd_fms_fine_step_dn", {&fmsMfdFmsFineDownCommand, false} },
	{ "/fms/mfd_fms_crsr", {&fmsMfdFmsCursorCommand, true} },
	{ "/fms/mfd_dir", {&fmsMfdFmsDirCommand, true} },
	{ "/fms/mfd_menu", {&fmsMfdFmsMenuCommand, true} },
	{ "/fms/mfd_fpl", {&fmsMfdFmsFplCommand, true} },
	{ "/fms/mfd_proc", {&fmsMfdFmsProcCommand, true} },

	{ "/fms/mfd_bksp", {&fmsMfdFmsBkspCommand, true} },
	{ "/fms/mfd_spc", {&fmsMfdFmsSpcCommand, true} },
	{ "/fms/mfd_clr", {&fmsMfdFmsClrCommand, true} },
	{ "/fms/mfd_ent", {&fmsMfdFmsEntCommand, true} },

	{ "/fms/kp_a", {&fmsMfdKpA, true} },
	{ "/fms/kp_b", {&fmsMfdKpB, true} },
	{ "/fms/kp_c", {&fmsMfdKpC, true} },
	{ "/fms/kp_d", {&fmsMfdKpD, true} },
	{ "/fms/kp_e", {&fmsMfdKpE, true} },
	{ "/fms/kp_f", {&fmsMfdKpF, true} },
	{ "/fms/kp_g", {&fmsMfdKpG, true} },
	{ "/fms/kp_h", {&fmsMfdKpH, true} },
	{ "/fms/kp_i", {&fmsMfdKpI, true} },
	{ "/fms/kp_j", {&fmsMfdKpJ, true} },
	{ "/fms/kp_k", {&fmsMfdKpK, true} },
	{ "/fms/kp_l", {&fmsMfdKpL, true} },
	{ "/fms/kp_m", {&fmsMfdKpM, true} },
	{ "/fms/kp_n", {&fmsMfdKpN, true} },
	{ "/fms/kp_o", {&fmsMfdKpO, true} },
	{ "/fms/kp_p", {&fmsMfdKpP, true} },
	{ "/fms/kp_q", {&fmsMfdKpQ, true} },
	{ "/fms/kp_r", {&fmsMfdKpR, true} },
	{ "/fms/kp_s", {&fmsMfdKpS, true} },
	{ "/fms/kp_t", {&fmsMfdKpT, true} },
	{ "/fms/kp_u", {&fmsMfdKpU, true} },
	{ "/fms/kp_v", {&fmsMfdKpV, true} },
	{ "/fms/kp_w", {&fmsMfdKpW, true} },
	{ "/fms/kp_x", {&fmsMfdKpX, true} },
	{ "/fms/kp_y", {&fmsMfdKpY, true} },
	{ "/fms/kp_z", {&fmsMfdKpZ, true} },

	{ "/fms/kp_1", {&fmsMfdKp1, true} },
	{ "/fms/kp_2", {&fmsMfdKp2, true} },
	{ "/fms/kp_3", {&fmsMfdKp3, true} },
	{ "/fms/kp_4", {&fmsMfdKp4, true} },
	{ "/fms/kp_5", {&fmsMfdKp5, true} },
	{ "/fms/kp_6", {&fmsMfdKp6, true} },
	{ "/fms/kp_7", {&fmsMfdKp7, true} },
	{ "/fms/kp_8", {&fmsMfdKp8, true} },
	{ "/fms/kp_9", {&fmsMfdKp9, true} },
	{ "/fms/kp_0", {&fmsMfdKp0, true} },
	{ "/fms/kp_pm", {&fmsMfdKpPm, true} },
	{ "/fms/kp_dot", {&fmsMfdKpDot, true} },

	{ "/fms/range_step_up", {&fmsMfdRangeUpCommand, false} },
	{ "/fms/range_step_dn", {&fmsMfdRangeDownCommand, false} },
	{ "/fms/pan", {&fmsMfdPanCommand, true} },
	{ "/fms/pan_u", {&fmsMfdPanUpCommand, true} },
	{ "/fms/pan_d", {&fmsMfdPanDownCommand, true} },
	{ "/fms/pan_l", {&fmsMfdPanLeftCommand, true} },
	{ "/fms/pan_r", {&fmsMfdPanRightCommand, true} },
	{ "/fms/pan_ul", {&fmsMfdPanUpLeftCommand, true} },
	{ "/fms/pan_ur", {&fmsMfdPanUpRightCommand, true} },
	{ "/fms/pan_dl", {&fmsMfdPanDownLeftCommand, true} },
	{ "/fms/pan_dr", {&fmsMfdPanDownRightCommand, true} },
};

static std::map<std::string, toggle_switch_t> toggleSwitchesByOscRef{
	{ "/util/crashbar", { &crashBarDownCommand, &crashBarUpCommand } },
	{ "/util/alt_static", {&altStaticToggleCommand, &altStaticToggleCommand} },
	{ "/ext/chocks", {&chocksToggleCommand, &chocksToggleCommand} },
	{ "/ext/gpu", {&gpuToggleCommand, &gpuToggleCommand} },
	{ "/ext/pilot_door/1/1", {&pilotDoorToggleCommand, &pilotDoorToggleCommand} },
	{ "/ext/pilot_door/1/2", {&pilotDoorLockToggleCommand, &pilotDoorLockToggleCommand} },
	{ "/ext/main_door_lock", {&mainDoorLockToggleCommand, &mainDoorLockToggleCommand} },
};

static std::map<std::string, direct_writable_t> directWritesByOscRef{
	{ "/util/lts_panel", {&ltsPanelDR, 0, 0, 1}},
	{ "/util/parking_brake", {&parkingBrakeDR,0, 0, 1 }},
	{ "/util/fan_spd", {&fanSpeedDR,0, 0, 1 }},
	{ "/util/temp", {&tempDR,0, 0, 1 }},
	{ "/util/hot_air", {&hotAirFlowDR,0, 0, 1 }},
	{ "/util/dump_guard", {&dumpGuardDR,0, 0, 1 }},
	{ "/ext/fuel_qty_l", {&fuelQtyLDR,0, 0, 1 }},
	{ "/ext/fuel_qty_r", {&fuelQtyRDR,0, 0, 1 }},
	{ "/ext/ties", {&tiesDR,0, 0, 1 }},
	{ "/ext/wick_l", {&wickCoverDR,2, 0, 2 }},
	{ "/ext/wick_r", {&wickCoverDR,2, 0, 2 }},
	{ "/ext/fuel_l", {&fuelCapDR,1, 0, 2 }},
	{ "/ext/fuel_r", {&fuelCapDR,1, 1, 2 }},
	{ "/ext/pitot_l", {&pitotCoverDR,2, 0, 2 }},
	{ "/ext/pitot_r", {&pitotCoverDR,2, 0, 2 }},
	{ "/ext/static_l", {&staticCoverDR,2, 0, 2 }},
	{ "/ext/static_r", {&staticCoverDR,2, 0, 2 }}
};

//static char mode[] = { '8', 'N','1',0 };

// Callbacks we will register when we create our window
float OscReadFlightLoop(float elapsedMe, float elapsedSim, int counter, void *refcon);
float OscWriteFlightLoop(float elapsedMe, float elapsedSim, int counter, void *refcon);

static std::queue<XPLMCommandRef *> commandQueue;
static std::mutex commandQueue_mutex;

static std::queue<direct_write_t> directWriteQueue;
static std::mutex directWriteQueue_mutex;

static std::queue<hold_t> holdQueue;
static std::mutex holdQueue_mutex;

static std::thread oscThread;
static int oscThreadCommandCount = 0;

class OscCommandPacketListener : public osc::OscPacketListener {
protected:

	void LogMessage(const osc::ReceivedMessage& m) {
		char buf[1024];
		float val;
		osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
		args >> val >> osc::EndMessage;
		sprintf(buf, "OSC MESSAGE RECEIVED: %s, %f\n", m.AddressPattern(), val);
		XPLMDebugString(buf);
	}

	virtual void ProcessMessage(const osc::ReceivedMessage& m,
		const IpEndpointName& remoteEndpoint)
	{
		(void)remoteEndpoint; // suppress unused parameter warning
		try {
			LogMessage(m);
			// example of parsing single messages. osc::OsckPacketListener
			// handles the bundle traversal.
			XPLMCommandRef *command;
			osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
			bool commandSet = false;
			const char *addressPattern = m.AddressPattern();
			std::string oscRef(addressPattern);
			if (std::strcmp(addressPattern, "/util/zone_sel/1/2") == 0) {
				float dir;
				args >> dir >> osc::EndMessage;
				if (dir > 0.0) {
					direct_write_t directWrite{ &tempZoneDR, 1.0f, 0 };
					std::lock_guard<std::mutex> guard(directWriteQueue_mutex);
					directWriteQueue.push(directWrite);
					return;
				}
			}
			else if (std::strcmp(addressPattern, "/util/zone_sel/1/1") == 0) {
				float dir;
				args >> dir >> osc::EndMessage;
				if (dir > 0.0) {
					direct_write_t directWrite{ &tempZoneDR, 0.0f, 0 };
					std::lock_guard<std::mutex> guard(directWriteQueue_mutex);
					directWriteQueue.push(directWrite);
					return;
				}
			}
			else if (std::strcmp(addressPattern, "/util/dump_act") == 0) {
				float dir;
				args >> dir >> osc::EndMessage;
				if (XPLMGetDatai(dumpGuardDR) > 0) {
					direct_write_t directWrite{ &dumpDR, dir, 0 };
					std::lock_guard<std::mutex> guard(directWriteQueue_mutex);
					directWriteQueue.push(directWrite);
					return;
				}
			}

			if (rotencsByOscRef.count(oscRef) > 0) {
				rotenc_t &rotenc = rotencsByOscRef[oscRef];
				float dir;
				args >> dir >> osc::EndMessage;

				int rotencCount = ++(rotenc.eventCount);
				if (rotencCount > rotenc.period) {
					rotenc.eventCount = 0;
					if (dir == 0.0) {
						command = rotenc.downCommand;
					}
					else {
						command = rotenc.upCommand;
					}
					commandSet = true;
				}
			}
			else if (buttonsByOscRef.count(oscRef) > 0) {
				button_t button = buttonsByOscRef[oscRef];
				float dir;
				args >> dir >> osc::EndMessage;
				if (button.held) {
					hold_t hold{ button.actuateCommand, dir > 0.0 };
					std::lock_guard<std::mutex> guard(holdQueue_mutex);
					holdQueue.push(hold);
					return;
				}
				else if (dir > 0.0) {
					command = button.actuateCommand;
					commandSet = true;
				}
			}
			else if (toggleSwitchesByOscRef.count(oscRef) > 0) {
				toggle_switch_t dswitch = toggleSwitchesByOscRef[oscRef];
				float dir;
				args >> dir >> osc::EndMessage;
				if (dir > 0.0 && dswitch.onCommand != NULL) {
					command = dswitch.onCommand;
					commandSet = true;
				}
				else if (dswitch.offCommand != NULL) {
					command = dswitch.offCommand;
					commandSet = true;
				}
			}
			else if (directWritesByOscRef.count(oscRef) > 0) {
				direct_writable_t directWritable = directWritesByOscRef[oscRef];
				float v;
				args >> v >> osc::EndMessage;
				direct_write_t directWrite{ directWritable.dataRef, v, directWritable.len, directWritable.index, directWritable.dRefSize };
				std::lock_guard<std::mutex> guard(directWriteQueue_mutex);
				directWriteQueue.push(directWrite);
				return;
			}
			if (commandSet) {
				std::lock_guard<std::mutex> guard(commandQueue_mutex);
				commandQueue.push(command);
			}
		}
		catch (osc::Exception& e) {
			// any parsing errors such as unexpected argument types, or 
			// missing arguments get thrown as exceptions.
			std::cout << "error while parsing message: "
				<< m.AddressPattern() << ": " << e.what() << "\n";
		}
	}
};

static OscCommandPacketListener listener;
static UdpListeningReceiveSocket *udpReceiveSocket;
UdpTransmitSocket *transmitSocket;
static int FLCBStartupFlag = 0;

void handleOscMessages() {
	udpReceiveSocket->Run();
}

static std::map<XPLMCommandRef, const char *> reverseCommandLookup;

void SetCommand(const char *key, XPLMCommandRef *var) {
	XPLMCommandRef command = XPLMFindCommand(key);
	*var = command;
	reverseCommandLookup[command] = key;
}

int SetFlapsUp(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRecon) {
	if (inPhase == xplm_CommandBegin) {
		XPLMSetDataf(flaprqstDR, 0.0f);
	}
	return 1;
}

int SetFlapsTakeoff(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRecon) {
	if (inPhase == xplm_CommandBegin) {
		XPLMSetDataf(flaprqstDR, 0.5f);
	}
	return 1;
}

int SetFlapsLanding(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRecon) {
	if (inPhase == xplm_CommandBegin) {
		XPLMSetDataf(flaprqstDR, 1.0f);
	}
	return 1;
}

static float lastNotch;

float DeferredInit(float elapsedMe, float elapsedSim, int counter, void *refcon) {
	if (FLCBStartupFlag == 0) {
		FLCBStartupFlag = 1;

		SetCommand("sim/GPS/g1000n1_baro_up",&baroUpCommand);
		SetCommand("sim/GPS/g1000n1_baro_down",&baroDownCommand);
		SetCommand("tbm900/actuators/efis/pfd1_baro_push",&baroStdCommand);
		SetCommand("tbm900/actuators/elec/emerg_handle_up",&crashBarUpCommand);
		SetCommand("tbm900/actuators/elec/emerg_handle_down",&crashBarDownCommand);
		SetCommand("tbm900/actuators/elec/source_gpu",&srcGpuCommand);
		SetCommand("tbm900/actuators/elec/source_batt",&srcBattCommand);
		SetCommand("tbm900/actuators/elec/source_off",&srcOffCommand);
		SetCommand("tbm900/actuators/elec/gen_stby",&genStbyCommand);
		SetCommand("tbm900/actuators/elec/gen_main",&genMainCommand);
		SetCommand("tbm900/actuators/elec/gen_off",&genOffCommand);
		SetCommand("tbm900/actuators/elec/main_gen_reset",&genResetMainCommand);
		SetCommand("tbm900/actuators/elec/stby_gen_reset",&genResetStbyCommand);
		SetCommand("tbm900/actuators/lights/landing_lights_ldg",&extLtsLdgCommand);
		SetCommand("tbm900/actuators/lights/landing_lights_taxi",&extLtsTaxiCommand);
		SetCommand("tbm900/actuators/lights/landing_lights_off",&extLtsOffCommand);
		SetCommand("tbm900/actuators/lights/pulse_syst_on",&ltsPulseOnCommand);
		SetCommand("tbm900/actuators/lights/pulse_syst_off",&ltsPulseOffCommand);
		SetCommand("sim/lights/nav_lights_on",&ltsNavOnCommand);
		SetCommand("sim/lights/nav_lights_off",&ltsNavOffCommand);
		SetCommand("sim/lights/strobe_lights_on",&ltsStrobeOnCommand);
		SetCommand("sim/lights/strobe_lights_off",&ltsStrobeOffCommand);
		SetCommand("tbm900/actuators/lights/dimmer_on",&ltsDimmerOnCommand);
		SetCommand("tbm900/actuators/lights/dimmer_off",&ltsDimmerOffCommand);
		SetCommand("tbm900/actuators/lights/cabin_on",&ltsCabinOnCommand);
		SetCommand("tbm900/actuators/lights/cabin_off",&ltsCabinOffCommand);
		SetCommand("tbm900/actuators/lights/access",&ltsAccessCommand);
		SetCommand("tbm900/actuators/elec/starter_up",&starterOnCommand);
		SetCommand("tbm900/actuators/elec/starter_down",&starterAbortCommand);
		SetCommand("tbm900/actuators/elec/ignition_auto",&ignitionAutoCommand);
		SetCommand("tbm900/actuators/elec/ignition_on",&ignitionOnCommand);
		SetCommand("tbm900/actuators/elec/ignition_off",&ignitionOffCommand);
		SetCommand("tbm900/actuators/elec/aux_bp_auto",&auxBpAutoCommand);
		SetCommand("tbm900/actuators/elec/aux_bp_on",&auxBpOnCommand);
		SetCommand("tbm900/actuators/elec/aux_bp_off",&auxBpOffCommand);
		SetCommand("tbm900/actuators/fuel/auto_man_off",&fuelSelAutoCommand);
		SetCommand("tbm900/actuators/fuel/auto_man_on",&fuelSelManualCommand);
		SetCommand("tbm900/actuators/fuel/shift",&fuelSelShiftCommand);
		SetCommand("tbm900/actuators/elec/ap_trims_on",&apTrimsOnCommand);
		SetCommand("tbm900/actuators/elec/ap_trims_ap_off",&apTrimsApOffCommand);
		SetCommand("tbm900/actuators/elec/ap_trims_off",&apTrimsOffCommand);
		SetCommand("tbm900/actuators/elec/elt_on",&eltOnCommand);
		SetCommand("tbm900/actuators/elec/elt_arm",&eltArmCommand);
		SetCommand("tbm900/actuators/elec/elt_test",&eltTestCommand);
		SetCommand("tbm900/actuators/ice/airframe_deice_on",&airframeDeiceOnCommand);
		SetCommand("tbm900/actuators/ice/airframe_deice_off",&airframeDeiceOffCommand);
		SetCommand("tbm900/actuators/ice/ice_lights_on",&iceLightOnCommand);
		SetCommand("tbm900/actuators/ice/ice_lights_off",&iceLightOffCommand);
		SetCommand("tbm900/actuators/ice/prop_deice_on",&propDeiceOnCommand);
		SetCommand("tbm900/actuators/ice/prop_deice_off",&propDeiceOffCommand);
		SetCommand("tbm900/actuators/ice/ws_heat_on",&windShieldOnCommand);
		SetCommand("tbm900/actuators/ice/ws_heat_off",&windShieldOffCommand);
		SetCommand("tbm900/actuators/ice/pitot_l_on",&pitotLOnCommand);
		SetCommand("tbm900/actuators/ice/pitot_l_off",&pitotLOffCommand);
		SetCommand("tbm900/actuators/ice/pitot_r_on",&pitotROnCommand);
		SetCommand("tbm900/actuators/ice/pitot_r_off",&pitotROffCommand);
		SetCommand("tbm900/actuators/ice/inert_sep_on",&inertSepOnCommand);
		SetCommand("tbm900/actuators/ice/inert_sep_off",&inertSepOffCommand);
		SetCommand("tbm900/actuators/ice/lts_test",&iceLightsTestCommand);
		SetCommand("sim/fuel/fuel_selector_lft",&setFuelLeftCommand);
		SetCommand("sim/fuel/fuel_selector_rgt",&setFuelRightCommand);
		SetCommand("sim/fuel/fuel_selector_none",&setFuelNoneCommand);
		SetCommand("sim/annunciator/clear_master_warning",&clearMasterWarningCommand);
		SetCommand("sim/annunciator/clear_master_caution",&clearMasterCautionCommand);
		SetCommand("sim/flight_controls/landing_gear_up",&gearUpCommand);
		SetCommand("sim/flight_controls/landing_gear_down",&gearDownCommand);
		SetCommand("tbm900/actuators/gear/lts_test",&gearLightsTestCommand);
		SetCommand("tbm900/actuators/gear/down_check",&gearCheckCommand);
		SetCommand("tbm900/actuators/ecs/ac_mode_auto",&acAutoCommand);
		SetCommand("tbm900/actuators/ecs/ac_mode_man",&acManualCommand);
		SetCommand("tbm900/actuators/ecs/ac_mode_off",&acOffCommand);
		SetCommand("tbm900/actuators/ecs/bleed_mode_auto",&bleedAutoCommand);
		SetCommand("tbm900/actuators/ecs/bleed_mode_off",&bleedOffCommand);
		SetCommand("tbm900/actuators/ecs/pres_mode_auto",&presModeAutoCommand);
		SetCommand("tbm900/actuators/ecs/pres_mode_maxdiff",&presModeMaxDiffCommand);
		SetCommand("sim/ice/alternate_static_port",&altStaticToggleCommand);
		SetCommand("tbm900/actuators/ecs/ram_air_intake",&ramAirToggleCommand);
		SetCommand("sim/electrical/GPU_toggle",&gpuToggleCommand);
		SetCommand("tbm900/actuators/gear/chocks",&chocksToggleCommand);
		SetCommand("tbm900/doors/pilot",&pilotDoorToggleCommand);
		SetCommand("tbm900/actuators/door_locks/pilot",&pilotDoorLockToggleCommand);
		SetCommand("tbm900/doors/main",&mainDoorToggleCommand);
		SetCommand("tbm900/actuators/door_locks/main",&mainDoorLockToggleCommand);
		SetCommand("tbm900/esi2000/softkey0", &stbyMCommand);
		SetCommand("tbm900/esi2000/softkey1", &stbySCommand);
		SetCommand("tbm900/esi2000/softkey2", &stbyMinusCommand);
		SetCommand("tbm900/esi2000/softkey3", &stbyPlusCommand);

		SetCommand("sim/GPS/g1000n1_hdg_up",&apHdgDirUpCommand);
		SetCommand("sim/GPS/g1000n1_hdg_down",&apHdgDirDownCommand);
		SetCommand("tbm900/actuators/ap/hdg_sync",&apHdgDirSyncCommand);
		SetCommand("tbm900/actuators/ap/hdg",&apHdgCommand);
		SetCommand("tbm900/actuators/ap/apr",&apAprCommand);
		SetCommand("tbm900/actuators/ap/bc",&apBcCommand);
		SetCommand("tbm900/actuators/ap/nav",&apNavCommand);
		SetCommand("tbm900/actuators/ap/fd",&apFdCommand);
		SetCommand("tbm900/actuators/ap/bank",&apBankCommand);
		SetCommand("sim/GPS/g1000n1_crs_up",&apCrs1UpCommand);
		SetCommand("sim/GPS/g1000n1_crs_down",&apCrs1DownCommand);
		SetCommand("tbm900/actuators/ap/crs1_dr",&apCrs1DirCommand);
		SetCommand("tbm900/actuators/ap/ap",&apApCommand);
		SetCommand("tbm900/actuators/ap/yd",&apYdCommand);
		SetCommand("tbm900/actuators/ap/xfr",&apXfrCommand);
		SetCommand("tbm900/actuators/ap/alt",&apAltCommand);
		SetCommand("sim/GPS/g1000n1_alt_outer_up",&apAltCoarseUpCommand);
		SetCommand("sim/GPS/g1000n1_alt_outer_down",&apAltCoarseDownCommand);
		SetCommand("sim/GPS/g1000n1_alt_inner_up",&apAltFineUpCommand);
		SetCommand("sim/GPS/g1000n1_alt_inner_down",&apAltFineDownCommand);
		SetCommand("tbm900/actuators/ap/vs",&apVsCommand);
		SetCommand("tbm900/actuators/ap/nose_up",&apNoseUpCommand);
		SetCommand("tbm900/actuators/ap/nose_down",&apNoseDownCommand);
		SetCommand("tbm900/actuators/ap/vnv",&apVnavCommand);
		SetCommand("tbm900/actuators/ap/flc",&apFlcCommand);
		SetCommand("tbm900/actuators/ap/spd",&apSpdCommand);
		SetCommand("sim/GPS/g1000n2_crs_up",&apCrs2UpCommand);
		SetCommand("sim/GPS/g1000n2_crs_down",&apCrs2DownCommand);
		SetCommand("tbm900/actuators/ap/crs2_dr",&apCrs2DirCommand);
		SetCommand("sim/GPS/g1000n1_nav_ff",&radNavSwapCommand);
		SetCommand("sim/GPS/g1000n1_nav12",&radNav12Command);
		SetCommand("sim/GPS/g1000n1_nav_outer_up",&radNavCoarseUpCommand);
		SetCommand("sim/GPS/g1000n1_nav_outer_down",&radNavCoarseDownCommand);
		SetCommand("sim/GPS/g1000n1_nav_inner_up",&radNavFineUpCommand);
		SetCommand("sim/GPS/g1000n1_nav_inner_down",&radNavFineDownCommand);
		SetCommand("tbm900/actuators/efis/pfd1_nav_vol_up",&radNavVolumeUpCommand);
		SetCommand("tbm900/actuators/efis/pfd1_nav_vol_down",&radNavVolumeDownCommand);
		SetCommand("sim/GPS/g1000n1_com_ff",&radComSwapCommand);
		SetCommand("sim/GPS/g1000n1_com12",&radCom12Command);
		SetCommand("sim/GPS/g1000n1_com_outer_up",&radComCoarseUpCommand);
		SetCommand("sim/GPS/g1000n1_com_outer_down",&radComCoarseDownCommand);
		SetCommand("sim/GPS/g1000n1_com_inner_up",&radComFineUpCommand);
		SetCommand("sim/GPS/g1000n1_com_inner_down",&radComFineDownCommand);
		SetCommand("tbm900/actuators/efis/pfd1_com_vol_up",&radComVolumeUpCommand);
		SetCommand("tbm900/actuators/efis/pfd1_com_vol_down",&radComVolumeDownCommand);
		SetCommand("tbm900/actuators/audio1/com1_mic",&audioMicCom1Command);
		SetCommand("tbm900/actuators/audio1/com1",&audioMonCom1Command);
		SetCommand("tbm900/actuators/audio1/com2_mic",&audioMicCom2Command);
		SetCommand("tbm900/actuators/audio1/com2",&audioMonCom2Command);
		SetCommand("tbm900/actuators/audio1/dme",&audioMonDmeCommand);
		SetCommand("tbm900/actuators/audio1/adf",&audioMonAdfCommand);
		SetCommand("tbm900/actuators/audio1/nav1",&audioMonNav1Command);
		SetCommand("tbm900/actuators/audio1/nav2",&audioMonNav2Command);
		SetCommand("tbm900/actuators/oxy/micro_mask_cover",&microMaskCoverToggleCommand);
		SetCommand("tbm900/actuators/oxy/micro_mask",&microMaskToggleCommand);
		SetCommand("sim/GPS/g1000n1_softkey1",&fmsPfdSK1Command);
		SetCommand("sim/GPS/g1000n1_softkey2",&fmsPfdSK2Command);
		SetCommand("sim/GPS/g1000n1_softkey3",&fmsPfdSK3Command);
		SetCommand("sim/GPS/g1000n1_softkey4",&fmsPfdSK4Command);
		SetCommand("sim/GPS/g1000n1_softkey5",&fmsPfdSK5Command);
		SetCommand("sim/GPS/g1000n1_softkey6",&fmsPfdSK6Command);
		SetCommand("sim/GPS/g1000n1_softkey7",&fmsPfdSK7Command);
		SetCommand("sim/GPS/g1000n1_softkey8",&fmsPfdSK8Command);
		SetCommand("sim/GPS/g1000n1_softkey9",&fmsPfdSK9Command);
		SetCommand("sim/GPS/g1000n1_softkey10",&fmsPfdSK10Command);
		SetCommand("sim/GPS/g1000n1_softkey11",&fmsPfdSK11Command);
		SetCommand("sim/GPS/g1000n1_softkey12",&fmsPfdSK12Command);
		SetCommand("sim/GPS/g1000n1_fms_outer_up",&fmsPfdFmsCoarseUpCommand);
		SetCommand("sim/GPS/g1000n1_fms_outer_down",&fmsPfdFmsCoarseDownCommand);
		SetCommand("sim/GPS/g1000n1_fms_inner_up",&fmsPfdFmsFineUpCommand);
		SetCommand("sim/GPS/g1000n1_fms_inner_down",&fmsPfdFmsFineDownCommand);
		SetCommand("sim/GPS/g1000n1_cursor",&fmsPfdFmsCursorCommand);
		SetCommand("sim/GPS/g1000n1_direct",&fmsPfdFmsDirCommand);
		SetCommand("sim/GPS/g1000n1_menu",&fmsPfdFmsMenuCommand);
		SetCommand("sim/GPS/g1000n1_fpl",&fmsPfdFmsFplCommand);
		SetCommand("sim/GPS/g1000n1_proc",&fmsPfdFmsProcCommand);
		SetCommand("sim/GPS/g1000n1_clr",&fmsPfdFmsClrCommand);
		SetCommand("sim/GPS/g1000n1_ent",&fmsPfdFmsEntCommand);
		SetCommand("sim/GPS/g1000n3_softkey1",&fmsMfdSK1Command);
		SetCommand("sim/GPS/g1000n3_softkey2",&fmsMfdSK2Command);
		SetCommand("sim/GPS/g1000n3_softkey3",&fmsMfdSK3Command);
		SetCommand("sim/GPS/g1000n3_softkey4",&fmsMfdSK4Command);
		SetCommand("sim/GPS/g1000n3_softkey5",&fmsMfdSK5Command);
		SetCommand("sim/GPS/g1000n3_softkey6",&fmsMfdSK6Command);
		SetCommand("sim/GPS/g1000n3_softkey7",&fmsMfdSK7Command);
		SetCommand("sim/GPS/g1000n3_softkey8",&fmsMfdSK8Command);
		SetCommand("sim/GPS/g1000n3_softkey9",&fmsMfdSK9Command);
		SetCommand("sim/GPS/g1000n3_softkey10",&fmsMfdSK10Command);
		SetCommand("sim/GPS/g1000n3_softkey11",&fmsMfdSK11Command);
		SetCommand("sim/GPS/g1000n3_softkey12",&fmsMfdSK12Command);
		SetCommand("tbm900/actuators/efis/mfd_keypad_fms_outer_up",&fmsMfdFmsCoarseUpCommand);
		SetCommand("tbm900/actuators/efis/mfd_keypad_fms_outer_down",&fmsMfdFmsCoarseDownCommand);
		SetCommand("sim/GPS/g1000n3_fms_inner_up",&fmsMfdFmsFineUpCommand);
		SetCommand("sim/GPS/g1000n3_fms_inner_down",&fmsMfdFmsFineDownCommand);
		SetCommand("sim/GPS/g1000n3_cursor",&fmsMfdFmsCursorCommand);
		SetCommand("sim/GPS/g1000n3_direct",&fmsMfdFmsDirCommand);
		SetCommand("sim/GPS/g1000n3_menu",&fmsMfdFmsMenuCommand);
		SetCommand("sim/GPS/g1000n3_fpl",&fmsMfdFmsFplCommand);
		SetCommand("sim/GPS/g1000n3_proc",&fmsMfdFmsProcCommand);
		SetCommand("tbm900/actuators/efis/mfd_keypad_clr",&fmsMfdFmsClrCommand);
		SetCommand("tbm900/actuators/efis/mfd_keypad_ent",&fmsMfdFmsEntCommand);
		SetCommand("tbm900/actuators/efis/mfd_keypad_bksp",&fmsMfdFmsBkspCommand);
		SetCommand("tbm900/actuators/efis/mfd_keypad_spc",&fmsMfdFmsSpcCommand);
		SetCommand("sim/GPS/g1000n3_range_up",&fmsMfdRangeUpCommand);
		SetCommand("sim/GPS/g1000n3_range_down",&fmsMfdRangeDownCommand);
		SetCommand("sim/GPS/g1000n3_pan_push",&fmsMfdPanCommand);
		SetCommand("sim/GPS/g1000n3_pan_up",&fmsMfdPanUpCommand);
		SetCommand("sim/GPS/g1000n3_pan_down",&fmsMfdPanDownCommand);
		SetCommand("sim/GPS/g1000n3_pan_left",&fmsMfdPanLeftCommand);
		SetCommand("sim/GPS/g1000n3_pan_right",&fmsMfdPanRightCommand);
		SetCommand("sim/GPS/g1000n3_pan_up_left",&fmsMfdPanUpLeftCommand);
		SetCommand("sim/GPS/g1000n3_pan_up_right",&fmsMfdPanUpRightCommand);
		SetCommand("sim/GPS/g1000n3_pan_down_left",&fmsMfdPanDownLeftCommand);
		SetCommand("sim/GPS/g1000n3_pan_down_right",&fmsMfdPanDownRightCommand);
		SetCommand("tbm900/actuators/efis/mfd_keypad_A",&fmsMfdKpA);
		SetCommand("tbm900/actuators/efis/mfd_keypad_B",&fmsMfdKpB);
		SetCommand("tbm900/actuators/efis/mfd_keypad_C",&fmsMfdKpC);
		SetCommand("tbm900/actuators/efis/mfd_keypad_D",&fmsMfdKpD);
		SetCommand("tbm900/actuators/efis/mfd_keypad_E",&fmsMfdKpE);
		SetCommand("tbm900/actuators/efis/mfd_keypad_F",&fmsMfdKpF);
		SetCommand("tbm900/actuators/efis/mfd_keypad_G",&fmsMfdKpG);
		SetCommand("tbm900/actuators/efis/mfd_keypad_H",&fmsMfdKpH);
		SetCommand("tbm900/actuators/efis/mfd_keypad_I",&fmsMfdKpI);
		SetCommand("tbm900/actuators/efis/mfd_keypad_J",&fmsMfdKpJ);
		SetCommand("tbm900/actuators/efis/mfd_keypad_K",&fmsMfdKpK);
		SetCommand("tbm900/actuators/efis/mfd_keypad_L",&fmsMfdKpL);
		SetCommand("tbm900/actuators/efis/mfd_keypad_M",&fmsMfdKpM);
		SetCommand("tbm900/actuators/efis/mfd_keypad_N",&fmsMfdKpN);
		SetCommand("tbm900/actuators/efis/mfd_keypad_O",&fmsMfdKpO);
		SetCommand("tbm900/actuators/efis/mfd_keypad_P",&fmsMfdKpP);
		SetCommand("tbm900/actuators/efis/mfd_keypad_Q",&fmsMfdKpQ);
		SetCommand("tbm900/actuators/efis/mfd_keypad_R",&fmsMfdKpR);
		SetCommand("tbm900/actuators/efis/mfd_keypad_S",&fmsMfdKpS);
		SetCommand("tbm900/actuators/efis/mfd_keypad_T",&fmsMfdKpT);
		SetCommand("tbm900/actuators/efis/mfd_keypad_U",&fmsMfdKpU);
		SetCommand("tbm900/actuators/efis/mfd_keypad_V",&fmsMfdKpV);
		SetCommand("tbm900/actuators/efis/mfd_keypad_W",&fmsMfdKpW);
		SetCommand("tbm900/actuators/efis/mfd_keypad_X",&fmsMfdKpX);
		SetCommand("tbm900/actuators/efis/mfd_keypad_Y",&fmsMfdKpY);
		SetCommand("tbm900/actuators/efis/mfd_keypad_Z",&fmsMfdKpZ);
		SetCommand("tbm900/actuators/efis/mfd_keypad_1",&fmsMfdKp1);
		SetCommand("tbm900/actuators/efis/mfd_keypad_2",&fmsMfdKp2);
		SetCommand("tbm900/actuators/efis/mfd_keypad_3",&fmsMfdKp3);
		SetCommand("tbm900/actuators/efis/mfd_keypad_4",&fmsMfdKp4);
		SetCommand("tbm900/actuators/efis/mfd_keypad_5",&fmsMfdKp5);
		SetCommand("tbm900/actuators/efis/mfd_keypad_6",&fmsMfdKp6);
		SetCommand("tbm900/actuators/efis/mfd_keypad_7",&fmsMfdKp7);
		SetCommand("tbm900/actuators/efis/mfd_keypad_8",&fmsMfdKp8);
		SetCommand("tbm900/actuators/efis/mfd_keypad_9",&fmsMfdKp9);
		SetCommand("tbm900/actuators/efis/mfd_keypad_0",&fmsMfdKp0);
		SetCommand("tbm900/actuators/efis/mfd_keypad_pm",&fmsMfdKpPm);
		SetCommand("tbm900/actuators/efis/mfd_keypad_period",&fmsMfdKpDot);
		SetCommand("sim/engines/mixture_down", &mixtureDownCommand);
		SetCommand("sim/engines/mixture_up", &mixtureUpCommand);

		baroSettingDR = XPLMFindDataRef("sim/cockpit/misc/barometer_setting");
		crashBarDR = XPLMFindDataRef("tbm900/switches/elec/emerg_handle");
		srcDR = XPLMFindDataRef("tbm900/switches/elec/source_raw");
		genDR = XPLMFindDataRef("tbm900/switches/elec/generator_raw");
		ltsExtDR = XPLMFindDataRef("tbm900/switches/lights/landing_lights_raw");
		ltsPulseDR = XPLMFindDataRef("tbm900/switches/lights/pulse_syst");
		ltsNavDR = XPLMFindDataRef("sim/cockpit2/switches/navigation_lights_on");
		ltsStrobeDR = XPLMFindDataRef("sim/cockpit2/switches/strobe_lights_on");
		ltsDimmerDR = XPLMFindDataRef("tbm900/switches/lights/dimmer");
		ltsCabinDR = XPLMFindDataRef("tbm900/switches/lights/cabin_lights");
		ltsPanelDR = XPLMFindDataRef("tbm900/knobs/lights/panel_brt");
		ignitionDR = XPLMFindDataRef("tbm900/switches/elec/ignition_raw");
		auxBpDR = XPLMFindDataRef("tbm900/switches/fuel/aux_bp_raw");
		fuelSelDR = XPLMFindDataRef("tbm900/switches/fuel/auto_man");
		apTrimsDR = XPLMFindDataRef("tbm900/switches/elec/ap_trims_raw");
		eltDR = XPLMFindDataRef("tbm900/switches/elec/elt");
		airframeDeiceDR = XPLMFindDataRef("tbm900/switches/ice/airframe_deice_raw");
		airframeDeiceLightsDR = XPLMFindDataRef("tbm900/lights/ice/airframe_deice");
		iceLightDR = XPLMFindDataRef("tbm900/switches/ice/ice_lights");
		propDeiceDR = XPLMFindDataRef("tbm900/switches/ice/prop_deice_raw");
		propDeiceLightDR = XPLMFindDataRef("tbm900/lights/ice/prop_deice");
		windShieldDR = XPLMFindDataRef("tbm900/switches/ice/ws_heat_raw");
		windShieldLightsDR = XPLMFindDataRef("tbm900/lights/ice/ws_heat");
		pitotLDR = XPLMFindDataRef("tbm900/switches/ice/pitot_l_raw");
		pitotRDR = XPLMFindDataRef("tbm900/switches/ice/pitot_r_raw");
		inertSepDR = XPLMFindDataRef("tbm900/switches/ice/inert_sep_raw");
		manFuelSelDR = XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_selector");
		masterCautionDR = XPLMFindDataRef("tbm900/lights/cas/master_caut");
		masterWarningDR = XPLMFindDataRef("tbm900/lights/cas/master_warn");
		parkingBrakeDR = XPLMFindDataRef("tbm900/switches/gear/park_brake");
		gearHandleDR = XPLMFindDataRef("sim/cockpit2/controls/gear_handle_down");
		gearSafeLightDR = XPLMFindDataRef("tbm900/lights/gear/lights");
		gearUnsafeLightDR = XPLMFindDataRef("tbm900/lights/gear/unsafe");
		acDR = XPLMFindDataRef("tbm900/switches/ecs/ac_mode");
		bleedDR = XPLMFindDataRef("tbm900/switches/ecs/bleed_mode");
		presModeDR = XPLMFindDataRef("tbm900/switches/ecs/pres_mode");
		dumpGuardDR = XPLMFindDataRef("tbm900/switches/ecs/dump_guard");
		dumpStateDR = XPLMFindDataRef("tbm900/lights/ecs/dump");
		dumpDR = XPLMFindDataRef("tbm900/switches/ecs/dump");
		fanSpeedDR = XPLMFindDataRef("tbm900/switches/ecs/fan_speed");
		tempDR = XPLMFindDataRef("tbm900/switches/ecs/temp_sel");
		tempZoneDR = XPLMFindDataRef("tbm900/switches/ecs/temp_zone");
		hotAirFlowDR = XPLMFindDataRef("tbm900/switches/ecs/hot_air");
		altStaticDR = XPLMFindDataRef("sim/cockpit2/switches/alternate_static_air_ratio");
		ramAirDR = XPLMFindDataRef("tbm900/switches/ecs/ram_air_intake");
		fuelQtyLDR = XPLMFindDataRef("sim/flightmodel/weight/m_fuel1");
		fuelQtyRDR = XPLMFindDataRef("sim/flightmodel/weight/m_fuel2");
		tiesDR = XPLMFindDataRef("tbm900/anim/engine/tied");
		wickCoverDR = XPLMFindDataRef("tbm900/anim/wing/wingtip_wick_flag");
		fuelCapDR = XPLMFindDataRef("tbm900/anim/wing/fuel_cap");
		pitotCoverDR = XPLMFindDataRef("tbm900/anim/wing/pitot_cover");
		chocksDR = XPLMFindDataRef("tbm900/switches/gear/chocks");
		staticCoverDR = XPLMFindDataRef("tbm900/anim/tail/static_cover");
		pilotDoorDR = XPLMFindDataRef("tbm900/doors/pilot");
		pilotDoorLockDR = XPLMFindDataRef("tbm900/anim/door_locks/pilot");
		mainDoorDR = XPLMFindDataRef("tbm900/doors/main");
		mainDoorLockDR = XPLMFindDataRef("tbm900/anim/door_locks/main");
		gpuDR = XPLMFindDataRef("tbm900/objects/misc/gpu");

		apHdgDirDR = XPLMFindDataRef("tbm900/knobs/ap/hdg");
		apCrs1DR = XPLMFindDataRef("sim/cockpit/gps/course");
		apAltDR = XPLMFindDataRef("tbm900/knobs/ap/alt");
		apVsDR = XPLMFindDataRef("sim/cockpit2/autopilot/vvi_dial_fpm");
		apFlcDR = XPLMFindDataRef("sim/cockpit2/autopilot/airspeed_dial_kts");
		apCrs2DR = XPLMFindDataRef("tbm900/knobs/ap/crs2");
		apHdgIndDR = XPLMFindDataRef("tbm900/lights/ap/hdg");
		apAprIndDR = XPLMFindDataRef("tbm900/lights/ap/apr");
		apBcIndDR = XPLMFindDataRef("tbm900/lights/ap/bc");
		apNavIndDR = XPLMFindDataRef("tbm900/lights/ap/nav");
		apBankIndDR = XPLMFindDataRef("tbm900/lights/ap/bank");
		apXfrLIndDR = XPLMFindDataRef("tbm900/lights/ap/comp_left");
		apXfrRIndDR = XPLMFindDataRef("tbm900/lights/ap/comp_right");
		apApIndDR = XPLMFindDataRef("tbm900/lights/ap/ap");
		apYdIndDR = XPLMFindDataRef("tbm900/lights/ap/yd");
		apAltIndDR = XPLMFindDataRef("tbm900/lights/ap/alt");
		apVsIndDR = XPLMFindDataRef("tbm900/lights/ap/vs");
		apVnavIndDR = XPLMFindDataRef("tbm900/lights/ap/vnv");
		apFlcIndDR = XPLMFindDataRef("tbm900/lights/ap/flc");
		audioMicCom1DR = XPLMFindDataRef("tbm900/lights/audio1/com1_mic");
		audioMonCom1DR = XPLMFindDataRef("tbm900/lights/audio1/com1");
		audioMicCom2DR = XPLMFindDataRef("tbm900/lights/audio1/com2_mic");
		audioMonCom2DR = XPLMFindDataRef("tbm900/lights/audio1/com2");
		audioMonDmeDR = XPLMFindDataRef("tbm900/lights/audio1/dme");
		audioMonAdfDR = XPLMFindDataRef("tbm900/lights/audio1/adf");
		audioMonNav1DR = XPLMFindDataRef("tbm900/lights/audio1/nav1");
		audioMonNav2DR = XPLMFindDataRef("tbm900/lights/audio1/nav2");
		radCom1FreqActiveDR = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_frequency_hz");
		radCom1FreqStbyDR = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_standby_frequency_hz");
		radCom2FreqActiveDR = XPLMFindDataRef("sim/cockpit2/radios/actuators/com2_frequency_hz");
		radCom2FreqStbyDR = XPLMFindDataRef("sim/cockpit2/radios/actuators/com2_standby_frequency_hz");
		radNav1FreqActiveDR = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_frequency_hz");
		radNav1FreqStbyDR = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_standby_frequency_hz");
		radNav2FreqActiveDR = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav2_frequency_hz");
		radNav2FreqStbyDR = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav2_standby_frequency_hz");
		altDR = XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot");
		kiasDR = XPLMFindDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot");
		vsDR = XPLMFindDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot");
		gsDR = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
		apfdDR = XPLMFindDataRef("sim/cockpit2/autopilot/autopilot_mode");
		flaprqstDR = XPLMFindDataRef("sim/flightmodel/controls/flaprqst");
		emergPowerDR = XPLMFindDataRef("tbm900/controls/engine/emerg_power");
		joystickAxisValuesDR = XPLMFindDataRef("sim/joystick/joy_mapped_axis_value");


		flapsUpCommand = XPLMCreateCommand("t9ctrl/actuators/flaps/set_flaps_up", "Sets the flaps to the UP position");
		flapsTakeoffCommand = XPLMCreateCommand("t9ctrl/actuators/flaps/set_flaps_to", "Sets the flaps to the TO position");
		flapsLandingCommand = XPLMCreateCommand("t9ctrl/actuators/flaps/set_flaps_ldg", "Sets the flaps to the LDG position");
		XPLMRegisterCommandHandler(flapsUpCommand, SetFlapsUp, false, NULL);
		XPLMRegisterCommandHandler(flapsTakeoffCommand, SetFlapsTakeoff, false, NULL);
		XPLMRegisterCommandHandler(flapsLandingCommand, SetFlapsLanding, false, NULL);

		float axisValues[53];
		XPLMGetDatavf(joystickAxisValuesDR, axisValues, 0, 50);
		lastNotch = axisValues[SMALL_THROTTLE_AXIS];

		transmitSocket = new UdpTransmitSocket(IpEndpointName("192.168.1.170", 9000));

		udpReceiveSocket = new UdpListeningReceiveSocket(
			IpEndpointName(IpEndpointName::ANY_ADDRESS, 8000),
			&listener);
		oscThread = std::thread(handleOscMessages);

		void *Param = NULL;
		XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
		if (PluginID != XPLM_NO_PLUGIN_ID) {
			XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"t9ctrl/timers/read");
			XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"t9ctrl/timers/write");
		}

		XPLMRegisterFlightLoopCallback(OscReadFlightLoop, 0.01f, NULL);
		XPLMRegisterFlightLoopCallback(OscWriteFlightLoop, 0.1f, NULL);
	}
	return 0;
}

static int readTime = 0;
static int writeTime = 0;

int getReadTime(void *inRefcon) {
	return readTime;
}

int getWriteTime(void *inRefcon) {
	return writeTime;
}

PLUGIN_API int XPluginStart(
	char *		outName,
	char *		outSig,
	char *		outDesc)
{
	strcpy(outName, "HelloWorld3Plugin");
	strcpy(outSig, "xpsdk.examples.helloworld3plugin");
	strcpy(outDesc, "A Hello World plug-in for the XPLM300 SDK.");

	XPLMRegisterFlightLoopCallback(DeferredInit, -1, NULL);
	readTimeDR = XPLMRegisterDataAccessor("t9ctrl/timers/read", xplmType_Int, 0, getReadTime, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
	writeTimeDR = XPLMRegisterDataAccessor("t9ctrl/timers/write", xplmType_Int, 0, getWriteTime, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
	if (FLCBStartupFlag == 1) {
		udpReceiveSocket->AsynchronousBreak();
		oscThread.join();
		udpReceiveSocket = NULL;
		XPLMUnregisterFlightLoopCallback(OscReadFlightLoop, NULL);
		XPLMUnregisterFlightLoopCallback(OscWriteFlightLoop, NULL);
	}
	XPLMUnregisterFlightLoopCallback(DeferredInit, NULL);
}

PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API int  XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) { }

static float commandfvs[5];
static int commandivs[5];

void LogCommand(XPLMCommandRef *command) {
		char buf[1024];
		const char*key = reverseCommandLookup[*command];
		sprintf(buf, "COMMAND EXECUTING: %s\n", key);
		XPLMDebugString(buf);
}

void executeCommands() {
	{
		std::lock_guard<std::mutex> guard(commandQueue_mutex);
		while (commandQueue.size() > 0) {
			//XPLMDebugString("EXECUTING COMMAND");
			XPLMCommandRef *command = commandQueue.front();
			commandQueue.pop();
			LogCommand(command);
			XPLMCommandOnce(*command);
		}
	}
	{
		std::lock_guard<std::mutex> guard(directWriteQueue_mutex);
		while (directWriteQueue.size() > 0) {
			direct_write_t directWrite = directWriteQueue.front();
			directWriteQueue.pop();
			XPLMDataRef dataRef = *directWrite.dataRef;
			int i;
			if (xplmType_IntArray & XPLMGetDataRefTypes(dataRef)) {
				for (i = 0; i < directWrite.len; i++) {
					commandivs[i] = (int)directWrite.value;
				}
				XPLMSetDatavi(dataRef, commandivs, directWrite.index, directWrite.len);
			}
			else if (xplmType_FloatArray & XPLMGetDataRefTypes(dataRef)) {
				for (i = 0; i < directWrite.len; i++) {
					commandfvs[i] = directWrite.value;
				}
				XPLMSetDatavf(dataRef, commandfvs, directWrite.index, directWrite.len);
			}
			else if (xplmType_Int & XPLMGetDataRefTypes(dataRef)) {
				XPLMSetDatai(dataRef, (int)directWrite.value);
			}
			else {
				XPLMSetDataf(dataRef, directWrite.value);
			}
		}
	}
	{
		std::lock_guard<std::mutex> guard(holdQueue_mutex);
		while (holdQueue.size() > 0) {
			hold_t hold = holdQueue.front();
			holdQueue.pop();
			XPLMCommandRef *command = hold.actuateCommand;
			if (hold.on) {
				XPLMCommandBegin(*command);
			}
			else {
				XPLMCommandEnd(*command);
			}
		}
	}
	float axisValues[53];
	XPLMGetDatavf(joystickAxisValuesDR, axisValues, 0, 50);
	XPLMSetDataf(emergPowerDR, (float)(1.0 - axisValues[EMERGENCY_OVERRIDE_AXIS]));
	float notchAxisValue = axisValues[SMALL_THROTTLE_AXIS];
	if (lastNotch != notchAxisValue) {
		//char buff[64];
		//sprintf(buff, "LAST NOTCH: %f, NOW NOTCH: %f\n");
		//XPLMDebugString(buff);
		if ((lastNotch < 0.25 && notchAxisValue >= 0.25) ||
			(lastNotch < 0.5 && notchAxisValue >= 0.5) ||
			(lastNotch < 0.75 && notchAxisValue >= 0.75)) {
			//XPLMDebugString("MIX DOWN\n");
			XPLMCommandOnce(mixtureDownCommand);
		}
		else if ((lastNotch >= 0.25 && notchAxisValue < 0.25) ||
				 (lastNotch >= 0.5 && notchAxisValue < 0.5) ||
				 (lastNotch >= 0.75 && notchAxisValue < 0.75)) {
			//XPLMDebugString("MIX UP\n");
			XPLMCommandOnce(mixtureUpCommand);
		}
	}
	lastNotch = notchAxisValue;
}

float OscReadFlightLoop(float elapsedMe, float elapsedSim, int counter, void *refcon)
{
	//auto start = std::chrono::high_resolution_clock::now();
	executeCommands();
	/*
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
90okkkkme = duration.count();
	*/
	return 0.01f;
}

static char oscBundleBuffer[OSC_BUNDLE_SIZE];

struct state_rep_t {
	XPLMDataRef *dataRef;
	const char *fmtStr;
	std::map<int, int> *valMap;
};

struct light_rep_t {
	XPLMDataRef *dataRef;
	const char *target[3];
	int size;
};

static std::vector<light_rep_t> lightReps{
	{&airframeDeiceLightsDR, {"/util/airframe_deice_ind_l", "/util/airframe_deice_ind_r", NULL}, 2},
	{&propDeiceLightDR, {"/util/prop_deice_ind", NULL, NULL}, 1},
	{&windShieldLightsDR, {"/util/wind_shield_ind_l", "/util/wind_shield_ind_r", NULL}, 2},
	{&masterCautionDR, {"/common/mc", NULL, NULL}, 1},
	{&masterWarningDR, {"/common/mw", NULL, NULL}, 1},
	{&gearSafeLightDR, {"/util/gear_state_n", "/util/gear_state_l", "/util/gear_state_r"}, 3},
	{&gearUnsafeLightDR, {"/util/gear_unsafe", NULL, NULL}, 1},
	{&tiesDR, {"/ext/ties", NULL, NULL}, 1},
	{&wickCoverDR, {"/ext/wick_l", "/ext/wick_r", NULL}, 2},
	{&pitotCoverDR, {"/ext/pitot_l", "/ext/pitot_r", NULL}, 2},
	{&staticCoverDR, {"/ext/static_l", "/ext/static_r", NULL}, 2},
	{&fuelCapDR, {"/ext/fuel_l", "/ext/fuel_r", NULL}, 2},
	{&chocksDR, {"/ext/chocks", NULL, NULL}, 1},
	{&gpuDR, {"/ext/gpu", NULL, NULL}, 1},
	{&pilotDoorDR, {"/ext/pilot_door/1/1", NULL, NULL}, 1},
	{&pilotDoorLockDR, {"/ext/pilot_door/1/2", NULL, NULL}, 1},
	{&mainDoorDR, {"/ext/main_door", NULL, NULL}, 1},
	{&mainDoorLockDR, {"/ext/main_door_lock", NULL, NULL}, 1},

	{&apHdgIndDR,  {"/radio_ap/hdg_ind", NULL, NULL}, 1},
	{&apAprIndDR,  {"/radio_ap/apr_ind", NULL, NULL}, 1},
	{&apBcIndDR,  {"/radio_ap/bc_ind", NULL, NULL}, 1},
	{&apNavIndDR,  {"/radio_ap/nav_ind", NULL, NULL}, 1},
	{&apBankIndDR,  {"/radio_ap/bank_ind", NULL, NULL}, 1},
	{&apXfrLIndDR,  {"/radio_ap/xfr_ind_l", NULL, NULL}, 1},
	{&apXfrRIndDR,  {"/radio_ap/xfr_ind_r", NULL, NULL}, 1},
	{&apApIndDR,  {"/radio_ap/ap_ind", NULL, NULL}, 1},
	{&apYdIndDR,  {"/radio_ap/yd_ind", NULL, NULL}, 1},
	{&apAltIndDR,  {"/radio_ap/alt_ind", NULL, NULL}, 1},
	{&apVsIndDR,  {"/radio_ap/vs_ind", NULL, NULL}, 1},
	{&apVnavIndDR,  {"/radio_ap/vnav_ind", NULL, NULL}, 1},
	{&apFlcIndDR,  {"/radio_ap/flc_ind", NULL, NULL}, 1},
	{&apHdgDirDR, {"/radio_ap/hdg_txt", NULL, NULL}, 1},
	{&apCrs1DR, {"/radio_ap/crs1_txt", NULL, NULL}, 1},
	{&apAltDR, {"/radio_ap/alt_sel_txt", NULL, NULL}, 1},
	{&apVsDR, {"/radio_ap/vs_sel_txt", NULL, NULL}, 1},
	{&apFlcDR, {"/radio_ap/flc_txt", NULL, NULL}, 1},
	{&apCrs2DR, {"/radio_ap/crs2_txt", NULL, NULL}, 1},
	{&audioMicCom1DR, {"/radio_ap/com1_mic_ind", NULL, NULL}, 1},
	{&audioMonCom1DR, {"/radio_ap/com1_mon_ind", NULL, NULL}, 1},
	{&audioMicCom2DR, {"/radio_ap/com2_mic_ind", NULL, NULL}, 1},
	{&audioMonCom2DR, {"/radio_ap/com2_mon_ind", NULL, NULL}, 1},
	{&audioMonDmeDR, {"/radio_ap/dme_mon_ind", NULL, NULL}, 1},
	{&audioMonAdfDR, {"/radio_ap/adf_mon_ind", NULL, NULL}, 1},
	{&audioMonNav1DR, {"/radio_ap/nav1_mon_ind", NULL, NULL}, 1},
	{&audioMonNav2DR, {"/radio_ap/nav2_mon_ind", NULL, NULL}, 1},
	{&radCom1FreqActiveDR, {"/radio_ap/com1_active_txt", NULL, NULL}, 1},
	{&radCom1FreqStbyDR, {"/radio_ap/com1_stby_txt", NULL, NULL}, 1},
	{&radCom2FreqActiveDR, {"/radio_ap/com2_active_txt", NULL, NULL}, 1},
	{&radCom2FreqStbyDR, {"/radio_ap/com2_stby_txt", NULL, NULL}, 1},
	{&radNav1FreqActiveDR, {"/radio_ap/nav1_active_txt", NULL, NULL}, 1},
	{&radNav1FreqStbyDR, {"/radio_ap/nav1_stby_txt", NULL, NULL}, 1},
	{&radNav2FreqActiveDR, {"/radio_ap/nav2_active_txt", NULL, NULL}, 1},
	{&radNav2FreqStbyDR, {"/radio_ap/nav2_stby_txt", NULL, NULL}, 1},
	{&altDR, {"/radio_ap/alt_txt", NULL, NULL}, 1},
	{&kiasDR, {"/radio_ap/kias_txt", NULL, NULL}, 1},
	{&vsDR, {"/radio_ap/vs_txt", NULL, NULL}, 1},
	{&gsDR, {"/radio_ap/gs_txt", NULL, NULL}, 1},
	{&apfdDR, {"/radio_ap/apfd_txt", NULL, NULL}, 1},
};

static std::map<int, int> manFuelSelValMap{
	{1,0},
	{0,1},
	{3,2}
};

static std::map<int, int> invertedValMap{
	{1, 0},
	{0, 1}
};

static std::vector<state_rep_t> stateReps{
	{&srcDR, "/util/power_src/%d/1", NULL},
	{&genDR, "/util/power_gen/%d/1", NULL},
	{&ltsExtDR , "/util/lts_ext/%d/1", NULL},
	{&ltsPulseDR , "/util/lts_pulse/%d/1", NULL},
	{&ltsNavDR , "/util/lts_nav/%d/1", NULL},
	{&ltsStrobeDR , "/util/lts_strobe/%d/1", NULL},
	{&ltsDimmerDR , "/util/lts_dimmer/%d/1", NULL},
	{&ltsCabinDR , "/util/lts_cabin/%d/1", NULL},
	{&ignitionDR , "/util/ignition/%d/1", NULL},
	{&auxBpDR , "/util/aux_bp/%d/1", NULL},
	{&fuelSelDR , "/util/fuel_sel/%d/1", &invertedValMap},
	{&apTrimsDR , "/util/ap_trims/%d/1", NULL},
	{&eltDR , "/util/elt/%d/1", NULL},
	{&airframeDeiceDR , "/util/airframe_deice/%d/1", NULL},
	{&iceLightDR , "/util/ice_light/%d/1", NULL},
	{&propDeiceDR , "/util/prop_deice/%d/1", NULL},
	{&windShieldDR , "/util/wind_shield/%d/1", NULL},
	{&pitotLDR , "/util/pitot_l/%d/1", NULL},
	{&pitotRDR , "/util/pitot_r/%d/1", NULL},
	{&inertSepDR , "/util/inert_sep/%d/1", NULL},
	{&manFuelSelDR, "/util/man_fuel_sel/1/%d", &manFuelSelValMap},
	{&gearHandleDR, "/util/gear/%d/1", &invertedValMap},
	{&acDR, "/util/ac/%d/1", NULL},
	{&bleedDR, "/util/bleed/%d/1", NULL},
	{&presModeDR, "/util/pres_mode/%d/1", NULL},
	{&tempZoneDR, "/util/zone_sel/1/%d", NULL}
};

static char *fuelQtyKeys[2]{
		"/ext/fuel_qty_l",
		"/ext/fuel_qty_r"
};

static char *fuelTxtKeys[2]{
	"/ext/fuel_txt_l",
	"/ext/fuel_txt_r"
};

static float kgsToGallons = 0.3246;
static float lightfvs[5];
static int lightivs[5];

void OscWrite() {
	osc::OutboundPacketStream p(oscBundleBuffer, OSC_BUNDLE_SIZE);

	p << osc::BeginBundleImmediate;
	p << osc::BeginMessage("/radio_ap/baro_txt") << XPLMGetDataf(baroSettingDR) << osc::EndMessage;
	p << osc::BeginMessage("/util/crashbar") << XPLMGetDataf(crashBarDR) << osc::EndMessage;
	p << osc::BeginMessage("/util/alt_static") << floor(XPLMGetDataf(altStaticDR)) << osc::EndMessage;
	p << osc::BeginMessage("/util/ram_air") << floor(XPLMGetDataf(ramAirDR)) << osc::EndMessage;
	p << osc::BeginMessage("/util/dump_guard") << XPLMGetDatai(dumpGuardDR) << osc::EndMessage;
	p << osc::BeginMessage("/util/dump") << XPLMGetDatai(dumpStateDR) << osc::EndMessage;
	int fuelCapStates[2];
	float fuelQtys[2]{
		XPLMGetDataf(fuelQtyLDR),
		XPLMGetDataf(fuelQtyRDR)
	};
	XPLMGetDatavi(fuelCapDR, fuelCapStates, 0, 2);

	int i = 0;
	for (i = 0; i < 2; i++) {
		if (fuelCapStates[i] > 0) {
			float fuelQty = fuelQtys[i];
			p << osc::BeginMessage(fuelQtyKeys[i]) << fuelQty << osc::EndMessage;
			char fuelBuf[6];
			sprintf(fuelBuf, "%.02f", fuelQty * kgsToGallons);
			p << osc::BeginMessage(fuelTxtKeys[i]) << fuelBuf << osc::EndMessage;
		}
		else {
			p << osc::BeginMessage(fuelQtyKeys[i]) << 0 << osc::EndMessage;
			p << osc::BeginMessage(fuelTxtKeys[i]) << "???" << osc::EndMessage;
		}
	}
	for (i = 0; i < stateReps.size(); i++) {
		state_rep_t stateRep = stateReps[i];
		XPLMDataRef *dataRef = stateRep.dataRef;
		int v = XPLMGetDatai(*dataRef);
		char buf[64];
		if (stateRep.valMap != NULL) {
			std::map<int, int> valMap = *stateRep.valMap;
			v = valMap[v];
		}
		sprintf(buf, stateRep.fmtStr, v + 1);
		p << osc::BeginMessage(buf) << 1 << osc::EndMessage;
	}
	for (i = 0; i < lightReps.size(); i++) {
		light_rep_t lightRep = lightReps[i];
		XPLMDataTypeID lightType = XPLMGetDataRefTypes(*lightRep.dataRef);
		if (lightRep.size > 1) {
			if (lightType & xplmType_FloatArray) {
				XPLMGetDatavf(*lightRep.dataRef, lightfvs, 0, lightRep.size);
				for (int k = 0; k < lightRep.size; k++) {
					lightivs[k] = (int)lightfvs[k];
				}
			}
			else {
				XPLMGetDatavi(*lightRep.dataRef, lightivs, 0, lightRep.size);
			}
			for (int j = 0; j < lightRep.size; j++) {
				p << osc::BeginMessage(lightRep.target[j]) << lightivs[j] << osc::EndMessage;
			}
		}
		else {
			int v;
			if (lightType & xplmType_Float) {
				float fv = XPLMGetDataf(*lightRep.dataRef);
				v = (int)fv;
			}
			else {
				v = XPLMGetDatai(*lightRep.dataRef);
			}
			p << osc::BeginMessage(lightRep.target[0]) << v << osc::EndMessage;
		}
	}

	p << osc::BeginMessage("/util/lts_panel") << XPLMGetDataf(ltsPanelDR) << osc::EndMessage;
	p << osc::BeginMessage("/util/parking_brake") << XPLMGetDataf(parkingBrakeDR) << osc::EndMessage;
	p << osc::BeginMessage("/util/fan_spd") << XPLMGetDatai(fanSpeedDR) << osc::EndMessage;
	p << osc::BeginMessage("/util/temp") << XPLMGetDataf(tempDR) << osc::EndMessage;
	p << osc::BeginMessage("/util/hot_air") << XPLMGetDataf(hotAirFlowDR) << osc::EndMessage;

	p << osc::EndBundle;

	transmitSocket->Send(p.Data(), p.Size());
}

float OscWriteFlightLoop(float elapsedMe, float elapsedSim, int counter, void *refcon)
{
	//auto start = std::chrono::high_resolution_clock::now();
	OscWrite();
	/*
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	writeTime = duration.count();
	*/
	return 0.1f;
}

