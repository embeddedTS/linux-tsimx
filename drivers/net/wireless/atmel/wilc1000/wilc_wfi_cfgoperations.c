/*!
*  @file	wilc_wfi_cfgopertaions.c
*  @brief	CFG80211 Function Implementation functionality
*  @author	aabouzaeid
*  			mabubakr
*  			mdaftedar
*  			zsalah
*  @sa		wilc_wfi_cfgopertaions.h top level OS wrapper file
*  @date	31 Aug 2010
*  @version	1.0
*/

#include "wilc_wfi_cfgoperations.h"
#include "wilc_wlan.c"
#include "linux_wlan.h"
#ifdef WILC_SDIO
#include "linux_wlan_sdio.h"    //tony : for set_wiphy_dev()
#endif


#define IS_MANAGMEMENT 				0x100
#define IS_MANAGMEMENT_CALLBACK 		0x080
#define IS_MGMT_STATUS_SUCCES			0x040
#define GET_PKT_OFFSET(a) (((a) >> 22) & 0x1ff)

extern void linux_wlan_free(void* vp);
extern int linux_wlan_get_firmware(perInterface_wlan_t* p_nic);
extern void linux_wlan_unlock(void* vp);
extern WILC_Uint16 Set_machw_change_vir_if(WILC_Bool bValue);

extern int mac_open(struct net_device *ndev);
extern int mac_close(struct net_device *ndev);

struct tstrNetworkInfo astrLastScannedNtwrksShadow[MAX_NUM_SCANNED_NETWORKS_SHADOW];
WILC_Uint32 u32LastScannedNtwrksCountShadow;
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
WILC_TimerHandle hDuringIpTimer;
#endif
WILC_TimerHandle hAgingTimer;
struct timer_list hEAPFrameBuffTimer;
static WILC_Uint8 op_ifcs=0;
extern WILC_Uint8 u8ConnectedSSID[6];

/*BugID_5137*/
WILC_Uint8 g_wilc_initialized = 1;
extern linux_wlan_t* g_linux_wlan;
#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
extern bool g_obtainingIP;
#endif

#define CHAN2G(_channel, _freq, _flags) {        \
	.band             = IEEE80211_BAND_2GHZ, \
	.center_freq      = (_freq),             \
	.hw_value         = (_channel),          \
	.flags            = (_flags),            \
	.max_antenna_gain = 0,                   \
	.max_power        = 30,                  \
}

/*Frequency range for channels*/
static struct ieee80211_channel WILC_WFI_2ghz_channels[] = {
	CHAN2G(1,  2412, 0),
	CHAN2G(2,  2417, 0),
	CHAN2G(3,  2422, 0),
	CHAN2G(4,  2427, 0),
	CHAN2G(5,  2432, 0),
	CHAN2G(6,  2437, 0),
	CHAN2G(7,  2442, 0),
	CHAN2G(8,  2447, 0),
	CHAN2G(9,  2452, 0),
	CHAN2G(10, 2457, 0),
	CHAN2G(11, 2462, 0),
	CHAN2G(12, 2467, 0),
	CHAN2G(13, 2472, 0),
	CHAN2G(14, 2484, 0),
};

#define RATETAB_ENT(_rate, _hw_value, _flags) { \
	.bitrate  = (_rate),                    \
	.hw_value = (_hw_value),                \
	.flags    = (_flags),                   \
}


/* Table 6 in section 3.2.1.1 */
static struct ieee80211_rate WILC_WFI_rates[] = {
	RATETAB_ENT(10,  0,  0),
	RATETAB_ENT(20,  1,  0),
	RATETAB_ENT(55,  2,  0),
	RATETAB_ENT(110, 3,  0),
	RATETAB_ENT(60,  9,  0),
	RATETAB_ENT(90,  6,  0),
	RATETAB_ENT(120, 7,  0),
	RATETAB_ENT(180, 8,  0),
	RATETAB_ENT(240, 9,  0),
	RATETAB_ENT(360, 10, 0),
	RATETAB_ENT(480, 11, 0),
	RATETAB_ENT(540, 12, 0),
};

#ifdef WILC_P2P
struct p2p_mgmt_data{
	int size;
	u8* buff;
};

/*Global variable used to state the current  connected STA channel*/
WILC_Uint8 u8WLANChannel = INVALID_CHANNEL ;

/*BugID_5442*/
WILC_Uint8 u8CurrChannel = 0;

WILC_Uint8  u8P2P_oui[] ={0x50,0x6f,0x9A,0x09};
WILC_Uint8 u8P2Plocalrandom=0x01;
WILC_Uint8 u8P2Precvrandom=0x00;
WILC_Uint8 u8P2P_vendorspec[]={0xdd,0x05,0x00,0x08,0x40,0x03};
WILC_Bool bWilc_ie=WILC_FALSE;
#endif

static struct ieee80211_supported_band WILC_WFI_band_2ghz = {
	.channels = WILC_WFI_2ghz_channels,
	.n_channels = ARRAY_SIZE(WILC_WFI_2ghz_channels),
	.bitrates = WILC_WFI_rates,
	.n_bitrates = ARRAY_SIZE(WILC_WFI_rates),
};


/*BugID_5137*/
struct add_key_params
{
	u8 key_idx;
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
	bool pairwise;
	#endif
	u8* mac_addr;
};
struct add_key_params g_add_gtk_key_params;
struct wilc_wfi_key g_key_gtk_params;
struct add_key_params g_add_ptk_key_params;
struct wilc_wfi_key g_key_ptk_params;
struct wilc_wfi_wep_key g_key_wep_params;
WILC_Uint8 g_flushing_in_progress = 0;
WILC_Bool g_ptk_keys_saved = WILC_FALSE;
WILC_Bool g_gtk_keys_saved = WILC_FALSE;
WILC_Bool g_wep_keys_saved = WILC_FALSE;

#define AGING_TIME	9*1000
#define duringIP_TIME 15000

void clear_shadow_scan(void* pUserVoid){
 	struct WILC_WFI_priv* priv;
	int i;
 	priv = (struct WILC_WFI_priv*)pUserVoid;
	if(op_ifcs == 0)
	{
		WILC_TimerDestroy(&hAgingTimer,WILC_NULL);
		PRINT_INFO(CORECONFIG_DBG, "destroy aging timer\n");

		for(i = 0; i < u32LastScannedNtwrksCountShadow; i++){
			if(astrLastScannedNtwrksShadow[u32LastScannedNtwrksCountShadow].pu8IEs != NULL)
			{
				WILC_FREE(astrLastScannedNtwrksShadow[i].pu8IEs);
				astrLastScannedNtwrksShadow[u32LastScannedNtwrksCountShadow].pu8IEs = NULL;
			}

			host_int_freeJoinParams(astrLastScannedNtwrksShadow[i].pJoinParams);
			astrLastScannedNtwrksShadow[i].pJoinParams = NULL;
		}
		u32LastScannedNtwrksCountShadow = 0;
	}

}

uint32_t get_rssi_avg(struct tstrNetworkInfo* pstrNetworkInfo)
{
	uint8_t i;
	int rssi_v = 0;
	uint8_t num_rssi = (pstrNetworkInfo->strRssi.u8Full)?NUM_RSSI:(pstrNetworkInfo->strRssi.u8Index);

	for(i=0;i<num_rssi;i++)
		rssi_v+=pstrNetworkInfo->strRssi.as8RSSI[i];

	rssi_v /= num_rssi;
	return rssi_v;
}

void refresh_scan(void* pUserVoid,uint8_t all,WILC_Bool bDirectScan){
 	struct WILC_WFI_priv* priv;
	struct wiphy* wiphy;
	struct cfg80211_bss* bss = NULL;
	int i;
	int rssi = 0;

 	priv = (struct WILC_WFI_priv*)pUserVoid;
	wiphy = priv->dev->ieee80211_ptr->wiphy;

	for(i = 0; i < u32LastScannedNtwrksCountShadow; i++)
	{
		struct tstrNetworkInfo* pstrNetworkInfo;
		pstrNetworkInfo = &(astrLastScannedNtwrksShadow[i]);


		if((!pstrNetworkInfo->u8Found) || all){
			WILC_Sint32 s32Freq;
			struct ieee80211_channel *channel;

			if(pstrNetworkInfo != WILC_NULL)
			{

				#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,38)
					s32Freq = ieee80211_channel_to_frequency((WILC_Sint32)pstrNetworkInfo->u8channel, IEEE80211_BAND_2GHZ);
				#else
					s32Freq = ieee80211_channel_to_frequency((WILC_Sint32)pstrNetworkInfo->u8channel);
				#endif

				channel = ieee80211_get_channel(wiphy, s32Freq);

				rssi = get_rssi_avg(pstrNetworkInfo);
				if(WILC_memcmp("DIRECT-", pstrNetworkInfo->au8ssid, 7) || bDirectScan)
				{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
					bss = cfg80211_inform_bss(wiphy, channel, CFG80211_BSS_FTYPE_UNKNOWN, pstrNetworkInfo->au8bssid, pstrNetworkInfo->u64Tsf, pstrNetworkInfo->u16CapInfo,
									pstrNetworkInfo->u16BeaconPeriod, (const u8*)pstrNetworkInfo->pu8IEs,
									(size_t)pstrNetworkInfo->u16IEsLen, (((WILC_Sint32)rssi) * 100), GFP_KERNEL);
#else
 					bss = cfg80211_inform_bss(wiphy, channel, pstrNetworkInfo->au8bssid, pstrNetworkInfo->u64Tsf, pstrNetworkInfo->u16CapInfo,
 									pstrNetworkInfo->u16BeaconPeriod, (const u8*)pstrNetworkInfo->pu8IEs,
 									(size_t)pstrNetworkInfo->u16IEsLen, (((WILC_Sint32)rssi) * 100), GFP_KERNEL);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
					cfg80211_put_bss(wiphy,bss);
#else
					cfg80211_put_bss(bss);
#endif
				}
			}

		}
	}

}

void reset_shadow_found(void* pUserVoid){
 	struct WILC_WFI_priv* priv;
	int i;
 	priv = (struct WILC_WFI_priv*)pUserVoid;
	for(i=0;i<u32LastScannedNtwrksCountShadow;i++){
		astrLastScannedNtwrksShadow[i].u8Found = 0;

		}
}

void update_scan_time(void* pUserVoid){
	struct WILC_WFI_priv* priv;
	int i;
 	priv = (struct WILC_WFI_priv*)pUserVoid;
	for(i=0;i<u32LastScannedNtwrksCountShadow;i++){
		astrLastScannedNtwrksShadow[i].u32TimeRcvdInScan = jiffies;
		}
}

void remove_network_from_shadow(void* pUserVoid){
 	struct WILC_WFI_priv* priv;
	unsigned long now = jiffies;
	int i,j;

 	priv = (struct WILC_WFI_priv*)pUserVoid;

	for(i=0;i<u32LastScannedNtwrksCountShadow;i++){
		if(time_after(now, astrLastScannedNtwrksShadow[i].u32TimeRcvdInScan + (unsigned long)(SCAN_RESULT_EXPIRE))){
			PRINT_D(CFG80211_DBG,"Network expired in ScanShadow: %s \n",astrLastScannedNtwrksShadow[i].au8ssid);

			if(astrLastScannedNtwrksShadow[i].pu8IEs != NULL)
			{
				WILC_FREE(astrLastScannedNtwrksShadow[i].pu8IEs);
				astrLastScannedNtwrksShadow[i].pu8IEs = NULL;
			}

			host_int_freeJoinParams(astrLastScannedNtwrksShadow[i].pJoinParams);

			for(j=i;(j<u32LastScannedNtwrksCountShadow-1);j++){
				astrLastScannedNtwrksShadow[j] = astrLastScannedNtwrksShadow[j+1];
			}
			u32LastScannedNtwrksCountShadow--;
		}
	}

	PRINT_D(CFG80211_DBG,"Number of cached networks: %d\n",u32LastScannedNtwrksCountShadow);
	if(u32LastScannedNtwrksCountShadow != 0)
		WILC_TimerStart(&(hAgingTimer), AGING_TIME, pUserVoid, WILC_NULL);
	else
		PRINT_D(CFG80211_DBG,"No need to restart Aging timer\n");
}

#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
void clear_duringIP(void* pUserVoid)
{
	PRINT_D(GENERIC_DBG,"GO:IP Obtained , enable scan\n");
	g_obtainingIP=WILC_FALSE;
}
#endif

int8_t is_network_in_shadow(struct tstrNetworkInfo* pstrNetworkInfo,void* pUserVoid){
 	struct WILC_WFI_priv* priv;
	int8_t state = -1;
	int i;

 	priv = (struct WILC_WFI_priv*)pUserVoid;
	if(u32LastScannedNtwrksCountShadow== 0){
		PRINT_D(CFG80211_DBG,"Starting Aging timer\n");
		WILC_TimerStart(&(hAgingTimer), AGING_TIME, pUserVoid, WILC_NULL);
		state = -1;
	}else{
		/* Linear search for now */
		for(i=0;i<u32LastScannedNtwrksCountShadow;i++){
			if(WILC_memcmp(astrLastScannedNtwrksShadow[i].au8bssid,
				  pstrNetworkInfo->au8bssid, 6) == 0){
								  state = i;
								  break;
			}
		}
	}
	return state;
}

void add_network_to_shadow(struct tstrNetworkInfo* pstrNetworkInfo,void* pUserVoid, void* pJoinParams){
 	struct WILC_WFI_priv* priv;
	int8_t ap_found = is_network_in_shadow(pstrNetworkInfo,pUserVoid);
	uint32_t ap_index = 0;
	uint8_t rssi_index = 0;
 	priv = (struct WILC_WFI_priv*)pUserVoid;

	if(u32LastScannedNtwrksCountShadow >= MAX_NUM_SCANNED_NETWORKS_SHADOW){
		PRINT_D(CFG80211_DBG,"Shadow network reached its maximum limit\n");
		return;
	}
	if(ap_found == -1){
			ap_index = u32LastScannedNtwrksCountShadow;
			u32LastScannedNtwrksCountShadow++;

		}else{
			ap_index = ap_found;
			}
		rssi_index = astrLastScannedNtwrksShadow[ap_index].strRssi.u8Index;
		astrLastScannedNtwrksShadow[ap_index].strRssi.as8RSSI[rssi_index++] = pstrNetworkInfo->s8rssi;
		if(rssi_index == NUM_RSSI)
		{
			rssi_index = 0;
			astrLastScannedNtwrksShadow[ap_index].strRssi.u8Full = 1;
		}
		astrLastScannedNtwrksShadow[ap_index].strRssi.u8Index = rssi_index;

		astrLastScannedNtwrksShadow[ap_index].s8rssi = pstrNetworkInfo->s8rssi;
		astrLastScannedNtwrksShadow[ap_index].u16CapInfo = pstrNetworkInfo->u16CapInfo;

		astrLastScannedNtwrksShadow[ap_index].u8SsidLen = pstrNetworkInfo->u8SsidLen;
		WILC_memcpy(astrLastScannedNtwrksShadow[ap_index].au8ssid,
				  	  pstrNetworkInfo->au8ssid, pstrNetworkInfo->u8SsidLen);

		WILC_memcpy(astrLastScannedNtwrksShadow[ap_index].au8bssid,
				  	  pstrNetworkInfo->au8bssid, ETH_ALEN);

		astrLastScannedNtwrksShadow[ap_index].u16BeaconPeriod = pstrNetworkInfo->u16BeaconPeriod;
		astrLastScannedNtwrksShadow[ap_index].u8DtimPeriod = pstrNetworkInfo->u8DtimPeriod;
		astrLastScannedNtwrksShadow[ap_index].u8channel = pstrNetworkInfo->u8channel;

		astrLastScannedNtwrksShadow[ap_index].u16IEsLen = pstrNetworkInfo->u16IEsLen;
		astrLastScannedNtwrksShadow[ap_index].u64Tsf = pstrNetworkInfo->u64Tsf;
	if(ap_found != -1)
		WILC_FREE(astrLastScannedNtwrksShadow[ap_index].pu8IEs);
		astrLastScannedNtwrksShadow[ap_index].pu8IEs =
			(WILC_Uint8*)WILC_MALLOC(pstrNetworkInfo->u16IEsLen); /* will be deallocated
																   by the WILC_WFI_CfgScan() function */
		WILC_memcpy(astrLastScannedNtwrksShadow[ap_index].pu8IEs,
				  	  pstrNetworkInfo->pu8IEs, pstrNetworkInfo->u16IEsLen);

		astrLastScannedNtwrksShadow[ap_index].u32TimeRcvdInScan = jiffies;
		astrLastScannedNtwrksShadow[ap_index].u32TimeRcvdInScanCached = jiffies;
		astrLastScannedNtwrksShadow[ap_index].u8Found = 1;
	if(ap_found != -1)
		host_int_freeJoinParams(astrLastScannedNtwrksShadow[ap_index].pJoinParams);
		astrLastScannedNtwrksShadow[ap_index].pJoinParams = pJoinParams;

}


/**
*  @brief 	CfgScanResult
*  @details  Callback function which returns the scan results found
*
*  @param[in] tenuScanEvent enuScanEvent: enum, indicating the scan event triggered, whether that is
*  			  SCAN_EVENT_NETWORK_FOUND or SCAN_EVENT_DONE
*  			  tstrNetworkInfo* pstrNetworkInfo: structure holding the scan results information
*  			  void* pUserVoid: Private structure associated with the wireless interface
*  @return 	NONE
*  @author	mabubakr
*  @date
*  @version	1.0
*/
static void CfgScanResult(enum tenuScanEvent enuScanEvent, struct tstrNetworkInfo* pstrNetworkInfo, void* pUserVoid, void* pJoinParams)
 {
 	struct WILC_WFI_priv* priv;
 	struct wiphy* wiphy;
 	WILC_Sint32 s32Freq;
 	struct ieee80211_channel *channel;
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct cfg80211_bss* bss = NULL;

 	priv = (struct WILC_WFI_priv*)pUserVoid;
 	if(priv->bCfgScanning == WILC_TRUE)
 	{
 		if(enuScanEvent == SCAN_EVENT_NETWORK_FOUND)
 		{
			wiphy = priv->dev->ieee80211_ptr->wiphy;
			WILC_NULLCHECK(s32Error, wiphy);
			if(wiphy->signal_type == CFG80211_SIGNAL_TYPE_UNSPEC
				&&
				( (((WILC_Sint32)pstrNetworkInfo->s8rssi) * 100) < 0
				||
				(((WILC_Sint32)pstrNetworkInfo->s8rssi) * 100) > 100)
				)
				{
					WILC_ERRORREPORT(s32Error, WILC_FAIL);
				}

			if(pstrNetworkInfo != WILC_NULL)
			{
				#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,38)
				s32Freq = ieee80211_channel_to_frequency((WILC_Sint32)pstrNetworkInfo->u8channel, IEEE80211_BAND_2GHZ);
				#else
				s32Freq = ieee80211_channel_to_frequency((WILC_Sint32)pstrNetworkInfo->u8channel);
				#endif
				channel = ieee80211_get_channel(wiphy, s32Freq);

				WILC_NULLCHECK(s32Error, channel);

				PRINT_INFO(CFG80211_DBG,"Network Info:: CHANNEL Frequency: %d, RSSI: %d, CapabilityInfo: %d,"
						"BeaconPeriod: %d \n",channel->center_freq, (((WILC_Sint32)pstrNetworkInfo->s8rssi) * 100),
						pstrNetworkInfo->u16CapInfo, pstrNetworkInfo->u16BeaconPeriod);

				if(pstrNetworkInfo->bNewNetwork == WILC_TRUE)
				{
					if(priv->u32RcvdChCount < MAX_NUM_SCANNED_NETWORKS) //TODO: mostafa: to be replaced by
																     //               max_scan_ssids
					{
						PRINT_D(CFG80211_DBG,"Network %s found\n",pstrNetworkInfo->au8ssid);


							priv->u32RcvdChCount++;



							if(pJoinParams == NULL){
								PRINT_INFO(CORECONFIG_DBG, ">> Something really bad happened\n");
							}
							add_network_to_shadow(pstrNetworkInfo,priv,pJoinParams);

							/*P2P peers are sent to WPA supplicant and added to shadow table*/

							if(!(WILC_memcmp("DIRECT-", pstrNetworkInfo->au8ssid, 7) ))
							{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
								bss = cfg80211_inform_bss(wiphy, channel, CFG80211_BSS_FTYPE_UNKNOWN,  pstrNetworkInfo->au8bssid, pstrNetworkInfo->u64Tsf, pstrNetworkInfo->u16CapInfo,
											pstrNetworkInfo->u16BeaconPeriod, (const u8*)pstrNetworkInfo->pu8IEs,
											(size_t)pstrNetworkInfo->u16IEsLen, (((WILC_Sint32)pstrNetworkInfo->s8rssi) * 100), GFP_KERNEL);
#else
 								bss = cfg80211_inform_bss(wiphy, channel, pstrNetworkInfo->au8bssid, pstrNetworkInfo->u64Tsf, pstrNetworkInfo->u16CapInfo,
 											pstrNetworkInfo->u16BeaconPeriod, (const u8*)pstrNetworkInfo->pu8IEs,
 											(size_t)pstrNetworkInfo->u16IEsLen, (((WILC_Sint32)pstrNetworkInfo->s8rssi) * 100), GFP_KERNEL);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
								cfg80211_put_bss(wiphy,bss);
#else
								cfg80211_put_bss(bss);
#endif
							}


						}
						else
						{
							PRINT_ER("Discovered networks exceeded the max limit\n");
						}
				}
				else
				{
					WILC_Uint32 i;
					/* So this network is discovered before, we'll just update its RSSI */
					for(i = 0; i < priv->u32RcvdChCount; i++)
					{
						if(WILC_memcmp(astrLastScannedNtwrksShadow[i].au8bssid, pstrNetworkInfo->au8bssid, 6) == 0)
						{
							PRINT_D(CFG80211_DBG,"Update RSSI of %s \n",astrLastScannedNtwrksShadow[i].au8ssid);

							astrLastScannedNtwrksShadow[i].s8rssi = pstrNetworkInfo->s8rssi;
							astrLastScannedNtwrksShadow[i].u32TimeRcvdInScan = jiffies;
							break;
						}
					}
				}
			}
 		}
 		else if(enuScanEvent == SCAN_EVENT_DONE)
 		{
 			PRINT_D(CFG80211_DBG,"Scan Done[%p] \n",priv->dev);
 			PRINT_D(CFG80211_DBG,"Refreshing Scan ... \n");
			refresh_scan(priv,1,WILC_FALSE);

 			if(priv->u32RcvdChCount > 0)
 			{
 				PRINT_D(CFG80211_DBG,"%d Network(s) found \n", priv->u32RcvdChCount);
 			}
 			else
 			{
 				PRINT_D(CFG80211_DBG,"No networks found \n");
 			}

			WILC_SemaphoreAcquire(&(priv->hSemScanReq), NULL);

			if(priv->pstrScanReq != WILC_NULL)
   			{
 				cfg80211_scan_done(priv->pstrScanReq, WILC_FALSE);
				priv->u32RcvdChCount = 0;
				priv->bCfgScanning = WILC_FALSE;
				priv->pstrScanReq = WILC_NULL;
			}
			WILC_SemaphoreRelease(&(priv->hSemScanReq), NULL);

 		}
		/*Aborting any scan operation during mac close*/
		else if(enuScanEvent == SCAN_EVENT_ABORTED)
 		{
			WILC_SemaphoreAcquire(&(priv->hSemScanReq), NULL);

	 		PRINT_D(CFG80211_DBG,"Scan Aborted \n");
			if(priv->pstrScanReq != WILC_NULL)
   			{

				update_scan_time(priv);
				refresh_scan(priv,1,WILC_FALSE);

				cfg80211_scan_done(priv->pstrScanReq,WILC_FALSE);
				priv->bCfgScanning = WILC_FALSE;
				priv->pstrScanReq = WILC_NULL;
  			}
			WILC_SemaphoreRelease(&(priv->hSemScanReq), NULL);
		}
 	}


	WILC_CATCH(s32Error)
	{
	}
 }


/**
*  @brief 	WILC_WFI_Set_PMKSA
*  @details  Check if pmksa is cached and set it.
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
int WILC_WFI_Set_PMKSA(WILC_Uint8 * bssid,struct WILC_WFI_priv* priv)
{
	WILC_Uint32 i;
	signed int s32Error = WILC_SUCCESS;


	for (i = 0; i < priv->pmkid_list.numpmkid; i++)
		{

			if (!WILC_memcmp(bssid,priv->pmkid_list.pmkidlist[i].bssid,
						ETH_ALEN))
			{
				PRINT_D(CFG80211_DBG,"PMKID successful comparison");

				/*If bssid is found, set the values*/
				s32Error = host_int_set_pmkid_info(priv->hWILCWFIDrv,&priv->pmkid_list);

				if(s32Error != WILC_SUCCESS)
					PRINT_ER("Error in pmkid\n");

				break;
			}
		}

	return s32Error;


}
int linux_wlan_set_bssid(struct net_device * wilc_netdev,uint8_t * pBSSID,uint8_t mode);


/**
*  @brief 	CfgConnectResult
*  @details
*  @param[in] tenuConnDisconnEvent enuConnDisconnEvent: Type of connection response either
*  			  connection response or disconnection notification.
*  			  tstrConnectInfo* pstrConnectInfo: COnnection information.
*  			  WILC_Uint8 u8MacStatus: Mac Status from firmware
*  			  tstrDisconnectNotifInfo* pstrDisconnectNotifInfo: Disconnection Notification
*  			  void* pUserVoid: Private data associated with wireless interface
*  @return 	NONE
*  @author	mabubakr
*  @date	01 MAR 2012
*  @version	1.0
*/
int connecting = 0;

static void CfgConnectResult(enum tenuConnDisconnEvent enuConnDisconnEvent,
							   struct tstrConnectInfo* pstrConnectInfo,
							   WILC_Uint8 u8MacStatus,
							   struct tstrDisconnectNotifInfo* pstrDisconnectNotifInfo,
							   void* pUserVoid)
{
	struct WILC_WFI_priv* priv;
	struct net_device* dev;
	#ifdef WILC_P2P
	struct WILC_WFIDrv * pstrWFIDrv;
	#endif
	WILC_Uint8 NullBssid[ETH_ALEN] = {0};
	connecting = 0;

	priv = (struct WILC_WFI_priv*)pUserVoid;
	dev = priv->dev;
#ifdef WILC_P2P
	pstrWFIDrv = (struct WILC_WFIDrv *)priv->hWILCWFIDrv;
#endif

	if(enuConnDisconnEvent == CONN_DISCONN_EVENT_CONN_RESP)
	{
		/*Initialization*/
		WILC_Uint16 u16ConnectStatus = WLAN_STATUS_SUCCESS;

		u16ConnectStatus = pstrConnectInfo->u16ConnectStatus;

		PRINT_D(CFG80211_DBG," Connection response received = %d\n",u8MacStatus);

		if((u8MacStatus == MAC_DISCONNECTED) &&
		    (pstrConnectInfo->u16ConnectStatus == SUCCESSFUL_STATUSCODE))
		{
			/* The case here is that our station was waiting for association response frame and has just received it containing status code
			    = SUCCESSFUL_STATUSCODE, while mac status is MAC_DISCONNECTED (which means something wrong happened) */
			u16ConnectStatus = WLAN_STATUS_UNSPECIFIED_FAILURE;
			linux_wlan_set_bssid(priv->dev,NullBssid,STATION_MODE);
			WILC_memset(u8ConnectedSSID,0,ETH_ALEN);

			/*BugID_5457*/
			/*Invalidate u8WLANChannel value on wlan0 disconnect*/
			#ifdef WILC_P2P
			if(!pstrWFIDrv->u8P2PConnect)
				u8WLANChannel = INVALID_CHANNEL;
			#endif

			PRINT_ER("Unspecified failure: Connection status %d : MAC status = %d \n",u16ConnectStatus,u8MacStatus);
		}

		if(u16ConnectStatus == WLAN_STATUS_SUCCESS)
		{
			WILC_Bool bNeedScanRefresh = WILC_FALSE;
			WILC_Uint32 i;

			PRINT_INFO(CFG80211_DBG,"Connection Successful:: BSSID: %x%x%x%x%x%x\n",pstrConnectInfo->au8bssid[0],
				pstrConnectInfo->au8bssid[1],pstrConnectInfo->au8bssid[2],pstrConnectInfo->au8bssid[3],pstrConnectInfo->au8bssid[4],pstrConnectInfo->au8bssid[5]);
			WILC_memcpy(priv->au8AssociatedBss, pstrConnectInfo->au8bssid, ETH_ALEN);

			//set bssid in frame filter
			//linux_wlan_set_bssid(dev,pstrConnectInfo->au8bssid,STATION_MODE);

			/* BugID_4209: if this network has expired in the scan results in the above nl80211 layer, refresh them here by calling
			    cfg80211_inform_bss() with the last Scan results before calling cfg80211_connect_result() to avoid
			    Linux kernel warning generated at the nl80211 layer */

			for(i = 0; i < u32LastScannedNtwrksCountShadow; i++)
			{
				if(WILC_memcmp(astrLastScannedNtwrksShadow[i].au8bssid,
								  pstrConnectInfo->au8bssid, ETH_ALEN) == 0)
				{
					unsigned long now = jiffies;

					if(time_after(now,
						astrLastScannedNtwrksShadow[i].u32TimeRcvdInScanCached + (unsigned long)(nl80211_SCAN_RESULT_EXPIRE - (1 * HZ))))
					{
						bNeedScanRefresh = WILC_TRUE;
					}

					break;
				}
			}

			if(bNeedScanRefresh == WILC_TRUE)
			{
				//RefreshScanResult(priv);
				/*BugID_5418*/
				/*Also, refrsh DIRECT- results if */
				refresh_scan(priv, 1,WILC_TRUE);

			}

		}


		PRINT_D(CFG80211_DBG,"Association request info elements length = %d\n", pstrConnectInfo->ReqIEsLen);

		PRINT_D(CFG80211_DBG,"Association response info elements length = %d\n", pstrConnectInfo->u16RespIEsLen);

		cfg80211_connect_result(dev, pstrConnectInfo->au8bssid,
								pstrConnectInfo->pu8ReqIEs, pstrConnectInfo->ReqIEsLen,
								pstrConnectInfo->pu8RespIEs, pstrConnectInfo->u16RespIEsLen,
								u16ConnectStatus, GFP_KERNEL); //TODO: mostafa: u16ConnectStatus to
															   // be replaced by pstrConnectInfo->u16ConnectStatus
	}
	else if(enuConnDisconnEvent == CONN_DISCONN_EVENT_DISCONN_NOTIF)
	{
		#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
		g_obtainingIP=WILC_FALSE;
		#endif
		PRINT_ER("Received MAC_DISCONNECTED from firmware with reason %d on dev [%p]\n",
		pstrDisconnectNotifInfo->u16reason, priv->dev);
		u8P2Plocalrandom=0x01;
		u8P2Precvrandom=0x00;
		bWilc_ie = WILC_FALSE;
		WILC_memset(priv->au8AssociatedBss, 0, ETH_ALEN);
		linux_wlan_set_bssid(priv->dev,NullBssid,STATION_MODE);
		WILC_memset(u8ConnectedSSID,0,ETH_ALEN);

		/*BugID_5457*/
		/*Invalidate u8WLANChannel value on wlan0 disconnect*/
		#ifdef WILC_P2P
		if(!pstrWFIDrv->u8P2PConnect)
			u8WLANChannel = INVALID_CHANNEL;
		#endif
		/*BugID_5315*/
		/*Incase "P2P CLIENT Connected" send deauthentication reason by 3 to force the WPA_SUPPLICANT to directly change
			virtual interface to station*/
		if((pstrWFIDrv->IFC_UP)&&(dev==g_linux_wlan->strInterfaceInfo[1].wilc_netdev))
		{
			pstrDisconnectNotifInfo->u16reason=3;
		}
		/*BugID_5315*/
		/*Incase "P2P CLIENT during connection(not connected)" send deauthentication reason by 1 to force the WPA_SUPPLICANT
			to scan again and retry the connection*/
		else if ((!pstrWFIDrv->IFC_UP)&&(dev==g_linux_wlan->strInterfaceInfo[1].wilc_netdev))
		{
			pstrDisconnectNotifInfo->u16reason=1;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)
		cfg80211_disconnected(dev, pstrDisconnectNotifInfo->u16reason, pstrDisconnectNotifInfo->ie,
						      pstrDisconnectNotifInfo->ie_len, false, GFP_KERNEL);
#else
		cfg80211_disconnected(dev, pstrDisconnectNotifInfo->u16reason, pstrDisconnectNotifInfo->ie,
						      pstrDisconnectNotifInfo->ie_len, GFP_KERNEL);
#endif
	}

}


/**
*  @brief 	WILC_WFI_CfgSetChannel
*  @details 	Set channel for a given wireless interface. Some devices
*      		may support multi-channel operation (by channel hopping) so cfg80211
*      		doesn't verify much. Note, however, that the passed netdev may be
*      		%NULL as well if the user requested changing the channel for the
*      		device itself, or for a monitor interface.
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
/*
*	struct changed from v3.8.0
*	tony, sswd, WILC-KR, 2013-10-29
*	struct cfg80211_chan_def {
         struct ieee80211_channel *chan;
         enum nl80211_chan_width width;
         u32 center_freq1;
         u32 center_freq2;
 	};
*/
static int WILC_WFI_CfgSetChannel(struct wiphy *wiphy,
		struct cfg80211_chan_def *chandef)
#else
static int WILC_WFI_CfgSetChannel(struct wiphy *wiphy,
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
		struct net_device *netdev,
	#endif
		struct ieee80211_channel *channel,
		enum nl80211_channel_type channel_type)
#endif
{

	WILC_Uint32 channelnum = 0;
	struct WILC_WFI_priv* priv;
	WILC_Sint32 s32Error = WILC_SUCCESS;
	priv = wiphy_priv(wiphy);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)	/* tony for v3.8.0 support */
	channelnum = ieee80211_frequency_to_channel(chandef->chan->center_freq);
	PRINT_D(CFG80211_DBG,"Setting channel %d with frequency %d\n", channelnum, chandef->chan->center_freq );
#else
	channelnum = ieee80211_frequency_to_channel(channel->center_freq);

	PRINT_D(CFG80211_DBG,"Setting channel %d with frequency %d\n", channelnum, channel->center_freq );
#endif

	u8CurrChannel = channelnum;
	s32Error   = host_int_set_mac_chnl_num(priv->hWILCWFIDrv,channelnum);

	if(s32Error != WILC_SUCCESS)
		PRINT_ER("Error in setting channel %d\n", channelnum);

	return s32Error;
}

/**
*  @brief 	WILC_WFI_CfgScan
*  @details 	Request to do a scan. If returning zero, the scan request is given
*      		the driver, and will be valid until passed to cfg80211_scan_done().
*      		For scan results, call cfg80211_inform_bss(); you can call this outside
*      		the scan/scan_done bracket too.
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mabubakr
*  @date	01 MAR 2012
*  @version	1.0
*/

/*
*	kernel version 3.8.8 supported
*	tony, sswd, WILC-KR, 2013-10-29
*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
static int WILC_WFI_CfgScan(struct wiphy *wiphy, struct cfg80211_scan_request *request)
#else
static int WILC_WFI_CfgScan(struct wiphy *wiphy,struct net_device *dev, struct cfg80211_scan_request *request)
#endif
{
	struct WILC_WFI_priv* priv;
	WILC_Uint32 i;
	WILC_Sint32 s32Error = WILC_SUCCESS;
	WILC_Uint8 au8ScanChanList[MAX_NUM_SCANNED_NETWORKS];
	struct tstrHiddenNetwork strHiddenNetwork;

	priv = wiphy_priv(wiphy);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)
	PRINT_D(CFG80211_DBG, "Scan on netdev [%p] host if [%x]\n",dev, (WILC_Uint32)priv->hWILCWFIDrv);
#endif

	/*if(connecting)
			return -EBUSY; */

	/*BugID_4800: if in AP mode, return.*/
	/*This check is to handle the situation when user*/
	/*requests "create group" during a running scan*/
	//host_int_set_wfi_drv_handler(priv->hWILCWFIDrv);
#if 0
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)	/* tony for v3.8.0 support */
	if(priv->dev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP)
	{
		PRINT_D(GENERIC_DBG,"Required scan while in AP mode");
		return s32Error;
	}
#else
	if(dev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP)
	{
		PRINT_D(GENERIC_DBG,"Required scan while in AP mode");
		s32Error = WILC_BUSY;
		return s32Error;
	}
#endif
#endif // end of if 0
	priv->pstrScanReq = request;

	priv->u32RcvdChCount = 0;




	reset_shadow_found(priv);

	priv->bCfgScanning = WILC_TRUE;
	if(request->n_channels <= MAX_NUM_SCANNED_NETWORKS) //TODO: mostafa: to be replaced by
														//               max_scan_ssids
	{
		for(i = 0; i < request->n_channels; i++)
		{
			au8ScanChanList[i] = (WILC_Uint8)ieee80211_frequency_to_channel(request->channels[i]->center_freq);
			PRINT_INFO(CFG80211_DBG, "ScanChannel List[%d] = %d,",i,au8ScanChanList[i]);
		}

		PRINT_D(CFG80211_DBG,"Requested num of scan channel %d\n",request->n_channels);
		PRINT_D(CFG80211_DBG,"Scan Request IE len =  %d\n",request->ie_len);

		PRINT_D(CFG80211_DBG,"Number of SSIDs %d\n",request->n_ssids);

		if(request->n_ssids >= 1)
		{


			strHiddenNetwork.pstrHiddenNetworkInfo = WILC_MALLOC(request->n_ssids * sizeof(struct tstrHiddenNetwork));
			strHiddenNetwork.u8ssidnum = request->n_ssids;


			/*BugID_4156*/
			for(i=0;i<request->n_ssids;i++)
			{

				if(request->ssids[i].ssid != NULL && request->ssids[i].ssid_len!=0)
				{
					strHiddenNetwork.pstrHiddenNetworkInfo[i].pu8ssid= WILC_MALLOC( request->ssids[i].ssid_len);
					WILC_memcpy(strHiddenNetwork.pstrHiddenNetworkInfo[i].pu8ssid,request->ssids[i].ssid,request->ssids[i].ssid_len);
					strHiddenNetwork.pstrHiddenNetworkInfo[i].u8ssidlen = request->ssids[i].ssid_len;
				}
				else
				{
							PRINT_D(CFG80211_DBG,"Received one NULL SSID \n");
							strHiddenNetwork.u8ssidnum-=1;
				}
			}
			PRINT_D(CFG80211_DBG,"Trigger Scan Request \n");
			s32Error = host_int_scan(priv->hWILCWFIDrv, USER_SCAN, ACTIVE_SCAN,
					  au8ScanChanList, request->n_channels,
					  (const WILC_Uint8*)request->ie, request->ie_len,
					  CfgScanResult, (void*)priv,&strHiddenNetwork);
		}
		else
		{
			PRINT_D(CFG80211_DBG,"Trigger Scan Request \n");
			s32Error = host_int_scan(priv->hWILCWFIDrv, USER_SCAN, ACTIVE_SCAN,
					  au8ScanChanList, request->n_channels,
					  (const WILC_Uint8*)request->ie, request->ie_len,
					  CfgScanResult, (void*)priv,NULL);
		}

	}
	else
	{
		PRINT_ER("Requested num of scanned channels is greater than the max, supported"
				  " channels \n");
	}

	if(s32Error != WILC_SUCCESS)
	{
		s32Error = -EBUSY;
		PRINT_ER(CFG80211_DBG,"Device is busy: Error(%d)\n",s32Error);
	}

	return s32Error;
}

/**
*  @brief 	WILC_WFI_CfgConnect
*  @details 	Connect to the ESS with the specified parameters. When connected,
*      		call cfg80211_connect_result() with status code %WLAN_STATUS_SUCCESS.
*      		If the connection fails for some reason, call cfg80211_connect_result()
*      		with the status from the AP.
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mabubakr
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_CfgConnect(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_connect_params *sme)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	WILC_Uint32 i;
	WILC_Uint32 chosen_bssid_index=u32LastScannedNtwrksCountShadow+1;
	//SECURITY_T  tenuSecurity_t = NO_SECURITY;
	WILC_Uint8 u8security = NO_ENCRYPT;
	enum AUTHTYPE tenuAuth_type = ANY;
	WILC_Char * pcgroup_encrypt_val;
	WILC_Char * pccipher_group;
	WILC_Char * pcwpa_version;

	struct WILC_WFI_priv* priv;
	struct WILC_WFIDrv * pstrWFIDrv;
	struct tstrNetworkInfo* pstrNetworkInfo = NULL;


	connecting = 1;
	priv = wiphy_priv(wiphy);
	pstrWFIDrv = (struct WILC_WFIDrv *)(priv->hWILCWFIDrv);


	PRINT_D(CFG80211_DBG,"Connecting to SSID [%s] on netdev [%p] host if [%x]\n",sme->ssid,dev, (WILC_Uint32)priv->hWILCWFIDrv);
	#ifdef WILC_P2P
	if(!(WILC_strncmp(sme->ssid,"DIRECT-", 7)))
	{
			PRINT_D(CFG80211_DBG,"Connected to Direct network,OBSS disabled\n");
			pstrWFIDrv->u8P2PConnect =1;
	}
	else
		pstrWFIDrv->u8P2PConnect= 0;
	#endif
	PRINT_INFO(CFG80211_DBG,"Required SSID = %s\n , AuthType = %d \n", sme->ssid,sme->auth_type);

	for(i = 0; i < u32LastScannedNtwrksCountShadow; i++)
	{
		if((sme->ssid_len == astrLastScannedNtwrksShadow[i].u8SsidLen) &&
			WILC_memcmp(astrLastScannedNtwrksShadow[i].au8ssid,
						sme->ssid,
						sme->ssid_len) == 0)
		{
			PRINT_INFO(CFG80211_DBG,"Network with required SSID is found %s\n", sme->ssid);
			if(sme->bssid == NULL)
			{
				/* BSSID is not passed from the user, so decision of matching
				 * is done by SSID only */
				PRINT_INFO(CFG80211_DBG,"BSSID is not passed from the user\n");
				/*
				 * Connect to the highest rssi with the required SSID in the shadow table 
				 * if the connection criteria is based only on the SSID
				 */
				if(chosen_bssid_index==(u32LastScannedNtwrksCountShadow+1))
				{
					/* For the first matching SSID, save its index */ 
					chosen_bssid_index=i;
				}
				else if(astrLastScannedNtwrksShadow[i].s8rssi>astrLastScannedNtwrksShadow[chosen_bssid_index].s8rssi)
				{
				   /* 
				    * For the next found matching SSID's, save their index if their RSSI is larger 
				 	* than the previously saved one
					*/ 
			    	chosen_bssid_index=i; 
				}
			}
			else
			{
				/* BSSID is also passed from the user, so decision of matching
				 * should consider also this passed BSSID */
				if(WILC_memcmp(astrLastScannedNtwrksShadow[i].au8bssid,
							       sme->bssid,
							       ETH_ALEN) == 0)
				{
					PRINT_INFO(CFG80211_DBG,"BSSID is passed from the user and matched\n");
					/* if the decision is based on the BSSID, there will be only one matching */
					chosen_bssid_index=i;
					break;
				}
			}
		}
	}

	if(chosen_bssid_index< u32LastScannedNtwrksCountShadow)
	{
		PRINT_D(CFG80211_DBG, "Required bss is in scan results\n");

		pstrNetworkInfo = &(astrLastScannedNtwrksShadow[chosen_bssid_index]);

		PRINT_INFO(CFG80211_DBG,"network BSSID to be associated: %x%x%x%x%x%x\n",
						pstrNetworkInfo->au8bssid[0], pstrNetworkInfo->au8bssid[1],
						pstrNetworkInfo->au8bssid[2], pstrNetworkInfo->au8bssid[3],
						pstrNetworkInfo->au8bssid[4], pstrNetworkInfo->au8bssid[5]);
	}
	else
	{
		s32Error = -ENOENT;
		if(u32LastScannedNtwrksCountShadow == 0)
			PRINT_D(CFG80211_DBG,"No Scan results yet\n");
		else
			PRINT_D(CFG80211_DBG,"Required bss not in scan results: Error(%d)\n",s32Error);

		goto done;
	}

	priv->WILC_WFI_wep_default = 0;
	WILC_memset(priv->WILC_WFI_wep_key, 0, sizeof(priv->WILC_WFI_wep_key));
	WILC_memset(priv->WILC_WFI_wep_key_len, 0, sizeof(priv->WILC_WFI_wep_key_len));

	PRINT_INFO(CFG80211_DBG,"sme->crypto.wpa_versions=%x\n", sme->crypto.wpa_versions);
	PRINT_INFO(CFG80211_DBG,"sme->crypto.cipher_group=%x\n", sme->crypto.cipher_group);

	PRINT_INFO(CFG80211_DBG,"sme->crypto.n_ciphers_pairwise=%d\n", sme->crypto.n_ciphers_pairwise);

	if(INFO)
	{
		for(i=0 ; i < sme->crypto.n_ciphers_pairwise ; i++)
			PRINT_D(CORECONFIG_DBG, "sme->crypto.ciphers_pairwise[%d]=%x\n", i, sme->crypto.ciphers_pairwise[i]);
	}

	if(sme->crypto.cipher_group != NO_ENCRYPT)
	{
		/* To determine the u8security value, first we check the group cipher suite then {in case of WPA or WPA2}
		    we will add to it the pairwise cipher suite(s) */
			pcwpa_version = "Default";
			PRINT_D(CORECONFIG_DBG, ">> sme->crypto.wpa_versions: %x\n",sme->crypto.wpa_versions);
			//case NL80211_WPA_VERSION_1:
			if (sme->crypto.cipher_group == WLAN_CIPHER_SUITE_WEP40)
			{
				//tenuSecurity_t = WEP_40;
				u8security = ENCRYPT_ENABLED | WEP;
				pcgroup_encrypt_val = "WEP40";
				pccipher_group = "WLAN_CIPHER_SUITE_WEP40";
				PRINT_INFO(CFG80211_DBG, "WEP Default Key Idx = %d\n",sme->key_idx);

				if(INFO)
				{
					for(i=0;i<sme->key_len;i++)
						PRINT_D(CORECONFIG_DBG, "WEP Key Value[%d] = %d\n", i, sme->key[i]);
				}
				priv->WILC_WFI_wep_default = sme->key_idx;
				priv->WILC_WFI_wep_key_len[sme->key_idx] = sme->key_len;
				WILC_memcpy(priv->WILC_WFI_wep_key[sme->key_idx], sme->key, sme->key_len);

				/*BugID_5137*/
				g_key_wep_params.key_len = sme->key_len;
				g_key_wep_params.key = WILC_MALLOC(sme->key_len);
				memcpy(g_key_wep_params.key, sme->key, sme->key_len);
				g_key_wep_params.key_idx = sme->key_idx;
				g_wep_keys_saved = WILC_TRUE;

				host_int_set_WEPDefaultKeyID(priv->hWILCWFIDrv,sme->key_idx);
				host_int_add_wep_key_bss_sta(priv->hWILCWFIDrv, sme->key,sme->key_len,sme->key_idx);
			}
			else if (sme->crypto.cipher_group == WLAN_CIPHER_SUITE_WEP104)
			{
				//tenuSecurity_t = WEP_104;
				u8security = ENCRYPT_ENABLED | WEP | WEP_EXTENDED;
				pcgroup_encrypt_val = "WEP104";
				pccipher_group = "WLAN_CIPHER_SUITE_WEP104";

				priv->WILC_WFI_wep_default = sme->key_idx;
				priv->WILC_WFI_wep_key_len[sme->key_idx] = sme->key_len;
				WILC_memcpy(priv->WILC_WFI_wep_key[sme->key_idx], sme->key, sme->key_len);

				/*BugID_5137*/
				g_key_wep_params.key_len = sme->key_len;
				g_key_wep_params.key = WILC_MALLOC(sme->key_len);
				memcpy(g_key_wep_params.key, sme->key, sme->key_len);
				g_key_wep_params.key_idx = sme->key_idx;
				g_wep_keys_saved = WILC_TRUE;

				host_int_set_WEPDefaultKeyID(priv->hWILCWFIDrv,sme->key_idx);
				host_int_add_wep_key_bss_sta(priv->hWILCWFIDrv, sme->key,sme->key_len,sme->key_idx);
			}
			else if( sme->crypto.wpa_versions & NL80211_WPA_VERSION_2)
			{
				//case NL80211_WPA_VERSION_2:
				if (sme->crypto.cipher_group == WLAN_CIPHER_SUITE_TKIP)
				{
					//tenuSecurity_t = WPA2_TKIP;
					u8security = ENCRYPT_ENABLED | WPA2 | TKIP;
					pcgroup_encrypt_val = "WPA2_TKIP";
					pccipher_group = "TKIP";
				}
				else //TODO: mostafa: here we assume that any other encryption type is AES
				{
					//tenuSecurity_t = WPA2_AES;
					u8security = ENCRYPT_ENABLED | WPA2 | AES;
					pcgroup_encrypt_val = "WPA2_AES";
					pccipher_group = "AES";
				}
				pcwpa_version = "WPA_VERSION_2";
			}
			else if( sme->crypto.wpa_versions & NL80211_WPA_VERSION_1)
			{
				if (sme->crypto.cipher_group == WLAN_CIPHER_SUITE_TKIP)
				{
					//tenuSecurity_t = WPA_TKIP;
					u8security = ENCRYPT_ENABLED | WPA | TKIP;
					pcgroup_encrypt_val = "WPA_TKIP";
					pccipher_group = "TKIP";
				}
				else //TODO: mostafa: here we assume that any other encryption type is AES
				{
					//tenuSecurity_t = WPA_AES;
					u8security = ENCRYPT_ENABLED | WPA | AES;
					pcgroup_encrypt_val = "WPA_AES";
					pccipher_group = "AES";

				}
				pcwpa_version = "WPA_VERSION_1";

				//break;
			}
			else
			{
				s32Error = -ENOTSUPP;
				PRINT_ER("Not supported cipher: Error(%d)\n",s32Error);

				goto done;
			}

		}

		/* After we set the u8security value from checking the group cipher suite, {in case of WPA or WPA2} we will
		     add to it the pairwise cipher suite(s) */
		if((sme->crypto.wpa_versions & NL80211_WPA_VERSION_1)
		    || (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2))
		{
			for(i=0 ; i < sme->crypto.n_ciphers_pairwise ; i++)
			{
				if (sme->crypto.ciphers_pairwise[i] == WLAN_CIPHER_SUITE_TKIP)
				{
					u8security = u8security | TKIP;
				}
				else //TODO: mostafa: here we assume that any other encryption type is AES
				{
					u8security = u8security | AES;
				}
			}
		}

	PRINT_D(CFG80211_DBG,"Adding key with cipher group = %x\n",sme->crypto.cipher_group);

	PRINT_D(CFG80211_DBG, "Authentication Type = %d\n",sme->auth_type);
	switch(sme->auth_type)
	{
		case NL80211_AUTHTYPE_OPEN_SYSTEM:
			PRINT_D(CFG80211_DBG, "In OPEN SYSTEM\n");
			tenuAuth_type = OPEN_SYSTEM;
			break;
		case NL80211_AUTHTYPE_SHARED_KEY:
			tenuAuth_type = SHARED_KEY;
   			PRINT_D(CFG80211_DBG, "In SHARED KEY\n");
			break;
		default:
			PRINT_D(CFG80211_DBG, "Automatic Authentation type = %d\n",sme->auth_type);
	}


	/* ai: key_mgmt: enterprise case */
	if (sme->crypto.n_akm_suites)
	{
		switch (sme->crypto.akm_suites[0])
		{
			case WLAN_AKM_SUITE_8021X:
				tenuAuth_type = IEEE8021;
				break;
			default:
				break;
		}
	}


	PRINT_INFO(CFG80211_DBG, "Required Channel = %d\n", pstrNetworkInfo->u8channel);

	PRINT_INFO(CFG80211_DBG, "Group encryption value = %s\n Cipher Group = %s\n WPA version = %s\n",
							      pcgroup_encrypt_val, pccipher_group,pcwpa_version);

	/*BugID_5442*/
	u8CurrChannel = pstrNetworkInfo->u8channel;

	if(!pstrWFIDrv->u8P2PConnect)
	{
		u8WLANChannel = pstrNetworkInfo->u8channel;
	}

	linux_wlan_set_bssid(dev,pstrNetworkInfo->au8bssid,STATION_MODE);

	s32Error = host_int_set_join_req(priv->hWILCWFIDrv, pstrNetworkInfo->au8bssid, sme->ssid,
							  	    sme->ssid_len, sme->ie, sme->ie_len,
							  	    CfgConnectResult, (void*)priv, u8security,
							  	    tenuAuth_type, pstrNetworkInfo->u8channel,
							  	    pstrNetworkInfo->pJoinParams);
	if(s32Error != WILC_SUCCESS)
	{
		PRINT_ER("host_int_set_join_req(): Error(%d) \n", s32Error);
		s32Error = -ENOENT;
		goto done;
	}

done:
	if(s32Error != WILC_SUCCESS)
	{
		PRINT_ER("%s(): Error(%d) \n",__FUNCTION__,s32Error);
		s32Error = -ENOENT;
		connecting = 0;
	}
	
	return s32Error;

}


/**
*  @brief 	WILC_WFI_disconnect
*  @details 	Disconnect from the BSS/ESS.
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_disconnect(struct wiphy *wiphy, struct net_device *dev,u16 reason_code)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	#ifdef WILC_P2P
	struct WILC_WFIDrv * pstrWFIDrv;
	#endif
	uint8_t NullBssid[ETH_ALEN] = {0};
	connecting = 0;
	priv = wiphy_priv(wiphy);

	/*BugID_5457*/
	/*Invalidate u8WLANChannel value on wlan0 disconnect*/
	#ifdef WILC_P2P
	pstrWFIDrv = (struct WILC_WFIDrv *)priv->hWILCWFIDrv;
	if(!pstrWFIDrv->u8P2PConnect)
		u8WLANChannel = INVALID_CHANNEL;
	#endif
	linux_wlan_set_bssid(priv->dev,NullBssid,STATION_MODE);

	PRINT_D(CFG80211_DBG,"Disconnecting with reason code(%d)\n", reason_code);

	u8P2Plocalrandom=0x01;
	u8P2Precvrandom=0x00;
	bWilc_ie = WILC_FALSE;
	#ifdef WILC_P2P
	pstrWFIDrv->p2p_mgmt_timeout = 0;
	#endif

	s32Error = host_int_disconnect(priv->hWILCWFIDrv, reason_code);
	if(s32Error != WILC_SUCCESS)
	{
		PRINT_ER("Error in disconnecting: Error(%d)\n", s32Error);
		s32Error = -EINVAL;
	}

	return s32Error;
}

/**
*  @brief 	WILC_WFI_add_key
*  @details 	Add a key with the given parameters. @mac_addr will be %NULL
*      		when adding a group key.
*  @param[in] key : key buffer; TKIP: 16-byte temporal key, 8-byte Tx Mic key, 8-byte Rx Mic Key
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_add_key(struct wiphy *wiphy, struct net_device *netdev,u8 key_index,
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
		bool pairwise,
	#endif
		const u8 *mac_addr,struct key_params *params)

{
	WILC_Sint32 s32Error = WILC_SUCCESS,KeyLen = params->key_len;
	WILC_Uint32 i;
	struct WILC_WFI_priv* priv;
	WILC_Uint8* pu8RxMic = NULL;
	WILC_Uint8* pu8TxMic = NULL;
	WILC_Uint8 u8mode = NO_ENCRYPT;
	#ifdef WILC_AP_EXTERNAL_MLME
	WILC_Uint8 u8gmode = NO_ENCRYPT;
	WILC_Uint8 u8pmode = NO_ENCRYPT;
	enum AUTHTYPE tenuAuth_type = ANY;
	#endif

	priv = wiphy_priv(wiphy);

	PRINT_D(CFG80211_DBG,"Adding key with cipher suite = %x\n",params->cipher);

	/*BugID_5137*/
	PRINT_D(CFG80211_DBG,"%x %x %d\n",(WILC_Uint32)wiphy, (WILC_Uint32)netdev, key_index);

	PRINT_D(CFG80211_DBG,"key %x %x %x\n",params->key[0],
										params->key[1],
										params->key[2]);


	switch(params->cipher)
		{
			case WLAN_CIPHER_SUITE_WEP40:
			case WLAN_CIPHER_SUITE_WEP104:
				#ifdef WILC_AP_EXTERNAL_MLME
				if(priv->wdev->iftype == NL80211_IFTYPE_AP)
				{
					priv->WILC_WFI_wep_key_len[key_index] = params->key_len;
					WILC_memcpy(priv->WILC_WFI_wep_key[key_index], params->key, params->key_len);

					PRINT_D(CFG80211_DBG, "Adding AP WEP Default key Idx = %d\n",key_index);
					PRINT_D(CFG80211_DBG, "Adding AP WEP Key len= %d\n",params->key_len);

					for(i=0;i<params->key_len;i++)
						PRINT_D(CFG80211_DBG, "WEP AP key val[%d] = %x\n", i, params->key[i]);

					tenuAuth_type = OPEN_SYSTEM;

					if(params->cipher == WLAN_CIPHER_SUITE_WEP40)
						u8mode = ENCRYPT_ENABLED | WEP;
					else
						u8mode = ENCRYPT_ENABLED | WEP | WEP_EXTENDED;

					host_int_add_wep_key_bss_ap(priv->hWILCWFIDrv,params->key,params->key_len,
						key_index,u8mode,tenuAuth_type);
					break;
				}
				#endif
				if(WILC_memcmp(params->key,priv->WILC_WFI_wep_key[key_index],params->key_len))
				{
					priv->WILC_WFI_wep_default = key_index;
					priv->WILC_WFI_wep_key_len[key_index] = params->key_len;
					WILC_memcpy(priv->WILC_WFI_wep_key[key_index], params->key, params->key_len);

					PRINT_D(CFG80211_DBG, "Adding WEP Default key Idx = %d\n",key_index);
					PRINT_D(CFG80211_DBG, "Adding WEP Key length = %d\n",params->key_len);
					if(INFO)
					{
						for(i=0;i<params->key_len;i++)
							PRINT_INFO(CFG80211_DBG, "WEP key value[%d] = %d\n", i, params->key[i]);
					}
					host_int_add_wep_key_bss_sta(priv->hWILCWFIDrv,params->key,params->key_len,
						key_index);
				}

				break;
			case WLAN_CIPHER_SUITE_TKIP:
			case WLAN_CIPHER_SUITE_CCMP:
				#ifdef WILC_AP_EXTERNAL_MLME
				if(priv->wdev->iftype == NL80211_IFTYPE_AP || priv->wdev->iftype == NL80211_IFTYPE_P2P_GO)
				{

					if(priv->wilc_gtk[key_index] == NULL)
						{
							priv->wilc_gtk[key_index] = (struct wilc_wfi_key *)WILC_MALLOC(sizeof(struct wilc_wfi_key));
							priv->wilc_gtk[key_index]->key=WILC_NULL;
							priv->wilc_gtk[key_index]->seq=WILC_NULL;

						}
					if(priv->wilc_ptk[key_index] == NULL)
						{
							priv->wilc_ptk[key_index] = (struct wilc_wfi_key *)WILC_MALLOC(sizeof(struct wilc_wfi_key));
							priv->wilc_ptk[key_index]->key=WILC_NULL;
							priv->wilc_ptk[key_index]->seq=WILC_NULL;
						}



					#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
					if (!pairwise)
			 		#else
				 	if (!mac_addr || is_broadcast_ether_addr(mac_addr))
					#endif
				 	{
					if(params->cipher == WLAN_CIPHER_SUITE_TKIP)
						u8gmode = ENCRYPT_ENABLED |WPA | TKIP;
					else
						u8gmode = ENCRYPT_ENABLED |WPA2 | AES;

					 priv->wilc_groupkey = u8gmode;

					if(params->key_len > 16 && params->cipher == WLAN_CIPHER_SUITE_TKIP)
				 	{

				 		pu8TxMic = params->key+24;
						pu8RxMic = params->key+16;
						KeyLen = params->key_len - 16;
				 	}
					/* if there has been previous allocation for the same index through its key, free that memory and allocate again*/
					if(priv->wilc_gtk[key_index]->key)
						WILC_FREE(priv->wilc_gtk[key_index]->key);

					priv->wilc_gtk[key_index]->key = (WILC_Uint8 *)WILC_MALLOC(params->key_len);
					WILC_memcpy(priv->wilc_gtk[key_index]->key,params->key,params->key_len);

					/* if there has been previous allocation for the same index through its seq, free that memory and allocate again*/
					if(priv->wilc_gtk[key_index]->seq)
						WILC_FREE(priv->wilc_gtk[key_index]->seq);

					if((params->seq_len)>0)
					{
						priv->wilc_gtk[key_index]->seq = (WILC_Uint8 *)WILC_MALLOC(params->seq_len);
						WILC_memcpy(priv->wilc_gtk[key_index]->seq,params->seq,params->seq_len);
					}

					priv->wilc_gtk[key_index]->cipher= params->cipher;
					priv->wilc_gtk[key_index]->key_len=params->key_len;
					priv->wilc_gtk[key_index]->seq_len=params->seq_len;

				if(INFO)
				{
					for(i=0;i<params->key_len;i++)
						PRINT_INFO(CFG80211_DBG, "Adding group key value[%d] = %x\n", i, params->key[i]);
					for(i=0;i<params->seq_len;i++)
						PRINT_INFO(CFG80211_DBG, "Adding group seq value[%d] = %x\n", i, params->seq[i]);
				}


					host_int_add_rx_gtk(priv->hWILCWFIDrv,params->key,KeyLen,
						key_index,params->seq_len,params->seq,pu8RxMic,pu8TxMic,AP_MODE,u8gmode);

				}
					else
					{
					     PRINT_INFO(CFG80211_DBG,"STA Address: %x%x%x%x%x\n",mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4]);

					if(params->cipher == WLAN_CIPHER_SUITE_TKIP)
						u8pmode = ENCRYPT_ENABLED | WPA | TKIP;
					else
						u8pmode = priv->wilc_groupkey | AES;


					if(params->key_len > 16 && params->cipher == WLAN_CIPHER_SUITE_TKIP)
					 {

					 	pu8TxMic = params->key+24;
						pu8RxMic = params->key+16;
						KeyLen = params->key_len - 16;
					 }

					if(priv->wilc_ptk[key_index]->key)
						WILC_FREE(priv->wilc_ptk[key_index]->key);

					priv->wilc_ptk[key_index]->key = (WILC_Uint8 *)WILC_MALLOC(params->key_len);

					if(priv->wilc_ptk[key_index]->seq)
						WILC_FREE(priv->wilc_ptk[key_index]->seq);

					if((params->seq_len)>0)
						priv->wilc_ptk[key_index]->seq = (WILC_Uint8 *)WILC_MALLOC(params->seq_len);

					if(INFO)
					{
					for(i=0;i<params->key_len;i++)
						PRINT_INFO(CFG80211_DBG, "Adding pairwise key value[%d] = %x\n", i, params->key[i]);

					for(i=0;i<params->seq_len;i++)
						PRINT_INFO(CFG80211_DBG, "Adding group seq value[%d] = %x\n", i, params->seq[i]);
					}

					WILC_memcpy(priv->wilc_ptk[key_index]->key,params->key,params->key_len);

					if((params->seq_len)>0)
					WILC_memcpy(priv->wilc_ptk[key_index]->seq,params->seq,params->seq_len);

					priv->wilc_ptk[key_index]->cipher= params->cipher;
					priv->wilc_ptk[key_index]->key_len=params->key_len;
					priv->wilc_ptk[key_index]->seq_len=params->seq_len;

					host_int_add_ptk(priv->hWILCWFIDrv,params->key,KeyLen,mac_addr,
					pu8RxMic,pu8TxMic,AP_MODE,u8pmode,key_index);
					}
					break;
				}
				#endif

				{
				u8mode=0;
			 #if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
				if (!pairwise)
			 #else
				 if (!mac_addr || is_broadcast_ether_addr(mac_addr))
			#endif
				 {
				 	if(params->key_len > 16 && params->cipher == WLAN_CIPHER_SUITE_TKIP)
				 	{
				 		//swap the tx mic by rx mic
				 		pu8RxMic = params->key+24;
						pu8TxMic = params->key+16;
						KeyLen = params->key_len - 16;
				 	}

					/*BugID_5137*/
					/*save keys only on interface 0 (wifi interface)*/
					if(!g_gtk_keys_saved && netdev == g_linux_wlan->strInterfaceInfo[0].wilc_netdev)
					{
						g_add_gtk_key_params.key_idx = key_index;
						#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
						g_add_gtk_key_params.pairwise = pairwise;
						#endif
						if(!mac_addr)
						{
							g_add_gtk_key_params.mac_addr = NULL;
						}
						else
						{
							g_add_gtk_key_params.mac_addr = WILC_MALLOC(ETH_ALEN);
							memcpy(g_add_gtk_key_params.mac_addr, mac_addr, ETH_ALEN);
						}
						g_key_gtk_params.key_len = params->key_len;
						g_key_gtk_params.seq_len = params->seq_len;
						g_key_gtk_params.key =  WILC_MALLOC(params->key_len);
						memcpy(g_key_gtk_params.key, params->key, params->key_len);
						if(params->seq_len > 0)
						{
							g_key_gtk_params.seq=  WILC_MALLOC(params->seq_len);
							memcpy(g_key_gtk_params.seq, params->seq, params->seq_len);
						}
						g_key_gtk_params.cipher = params->cipher;

						PRINT_D(CFG80211_DBG,"key %x %x %x\n",g_key_gtk_params.key[0],
															g_key_gtk_params.key[1],
															g_key_gtk_params.key[2]);
						g_gtk_keys_saved = WILC_TRUE;
					}

					host_int_add_rx_gtk(priv->hWILCWFIDrv,params->key,KeyLen,
						key_index,params->seq_len,params->seq,pu8RxMic,pu8TxMic,STATION_MODE,u8mode);
					//host_int_add_tx_gtk(priv->hWILCWFIDrv,params->key_len,params->key,key_index);
				 }
				 else
				 {
				 	if(params->key_len > 16 && params->cipher == WLAN_CIPHER_SUITE_TKIP)
					 	{
					 		//swap the tx mic by rx mic
					 		pu8RxMic = params->key+24;
							pu8TxMic = params->key+16;
							KeyLen = params->key_len - 16;
					 	}

					/*BugID_5137*/
					/*save keys only on interface 0 (wifi interface)*/
					if(!g_ptk_keys_saved && netdev == g_linux_wlan->strInterfaceInfo[0].wilc_netdev)
					{
						g_add_ptk_key_params.key_idx = key_index;
						#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
						g_add_ptk_key_params.pairwise = pairwise;
						#endif
						if(!mac_addr)
						{
							g_add_ptk_key_params.mac_addr = NULL;
						}
						else
						{
							g_add_ptk_key_params.mac_addr = WILC_MALLOC(ETH_ALEN);
							memcpy(g_add_ptk_key_params.mac_addr, mac_addr, ETH_ALEN);
						}
						g_key_ptk_params.key_len = params->key_len;
						g_key_ptk_params.seq_len = params->seq_len;
						g_key_ptk_params.key =  WILC_MALLOC(params->key_len);
						memcpy(g_key_ptk_params.key, params->key, params->key_len);
						if(params->seq_len > 0)
						{
							g_key_ptk_params.seq=  WILC_MALLOC(params->seq_len);
							memcpy(g_key_ptk_params.seq, params->seq, params->seq_len);
						}
						g_key_ptk_params.cipher = params->cipher;

						PRINT_D(CFG80211_DBG,"key %x %x %x\n",g_key_ptk_params.key[0],
															g_key_ptk_params.key[1],
															g_key_ptk_params.key[2]);
						g_ptk_keys_saved = WILC_TRUE;
					}

				 	host_int_add_ptk(priv->hWILCWFIDrv,params->key,KeyLen,mac_addr,
					pu8RxMic,pu8TxMic,STATION_MODE,u8mode,key_index);
					 PRINT_D(CFG80211_DBG,"Adding pairwise key\n");
					if(INFO)
					{
						 for(i=0;i<params->key_len;i++)
							PRINT_INFO(CFG80211_DBG, "Adding pairwise key value[%d] = %d\n", i, params->key[i]);
					}
				 }
				}
				break;
			default:
				PRINT_ER("Not supported cipher: Error(%d)\n",s32Error);
				s32Error = -ENOTSUPP;

		}

	return s32Error;
}

/**
*  @brief 	WILC_WFI_del_key
*  @details 	Remove a key given the @mac_addr (%NULL for a group key)
*      		and @key_index, return -ENOENT if the key doesn't exist.
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_del_key(struct wiphy *wiphy, struct net_device *netdev,
	u8 key_index,
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
		bool pairwise,
	#endif
		const u8 *mac_addr)
{
	struct WILC_WFI_priv* priv;
	WILC_Sint32 s32Error = WILC_SUCCESS;

	priv = wiphy_priv(wiphy);

		/*BugID_5137*/
		/*delete saved keys, if any*/
		if(netdev == g_linux_wlan->strInterfaceInfo[0].wilc_netdev)
		{
			g_ptk_keys_saved = WILC_FALSE;
			g_gtk_keys_saved = WILC_FALSE;
			g_wep_keys_saved = WILC_FALSE;

			/*Delete saved WEP keys params, if any*/
			if(g_key_wep_params.key != NULL)
			{
				WILC_FREE(g_key_wep_params.key);
				g_key_wep_params.key = NULL;
			}

	/*freeing memory allocated by "wilc_gtk" and "wilc_ptk" in "WILC_WIFI_ADD_KEY"*/

	#ifdef WILC_AP_EXTERNAL_MLME
	if((priv->wilc_gtk[key_index])!=NULL)
	{

		if(priv->wilc_gtk[key_index]->key!=NULL)
		{

			WILC_FREE(priv->wilc_gtk[key_index]->key);
			priv->wilc_gtk[key_index]->key=NULL;
		}
		if(priv->wilc_gtk[key_index]->seq)
		{

			WILC_FREE(priv->wilc_gtk[key_index]->seq);
			priv->wilc_gtk[key_index]->seq=NULL;
		}

		WILC_FREE(priv->wilc_gtk[key_index]);
		priv->wilc_gtk[key_index]=NULL;

	}

	if((priv->wilc_ptk[key_index])!=NULL)
	{

		if(priv->wilc_ptk[key_index]->key)
		{

			WILC_FREE(priv->wilc_ptk[key_index]->key);
			priv->wilc_ptk[key_index]->key=NULL;
		}
		if(priv->wilc_ptk[key_index]->seq)
		{

			WILC_FREE(priv->wilc_ptk[key_index]->seq);
			priv->wilc_ptk[key_index]->seq=NULL;
		}
		WILC_FREE(priv->wilc_ptk[key_index]);
		priv->wilc_ptk[key_index]=NULL;
	}
	#endif

			/*Delete saved PTK and GTK keys params, if any*/
			if(g_key_ptk_params.key != NULL)
			{
				WILC_FREE(g_key_ptk_params.key);
				g_key_ptk_params.key = NULL;
			}
			if(g_key_ptk_params.seq != NULL)
			{
				WILC_FREE(g_key_ptk_params.seq);
				g_key_ptk_params.seq = NULL;
			}

			if(g_key_gtk_params.key != NULL)
			{
				WILC_FREE(g_key_gtk_params.key);
				g_key_gtk_params.key = NULL;
			}
			if(g_key_gtk_params.seq != NULL)
			{
				WILC_FREE(g_key_gtk_params.seq);
				g_key_gtk_params.seq = NULL;
			}

		}

		if(key_index >= 0 && key_index <= 3)
		{
			WILC_memset(priv->WILC_WFI_wep_key[key_index], 0, priv->WILC_WFI_wep_key_len[key_index]);
			priv->WILC_WFI_wep_key_len[key_index] = 0;

			PRINT_D(CFG80211_DBG, "Removing WEP key with index = %d\n",key_index);
			s32Error = host_int_remove_wep_key(priv->hWILCWFIDrv,key_index);
		}
		else
		{
			PRINT_D(CFG80211_DBG, "Removing all installed keys\n");
			s32Error = host_int_remove_key(priv->hWILCWFIDrv,mac_addr);
		}
		
		/* return the error code which the sublicant will understand to the sublicant  */
		if(s32Error)
		{
			s32Error = -EINVAL; /* Invalid argument */
		}

	return s32Error;
}

/**
*  @brief 	WILC_WFI_get_key
*  @details 	Get information about the key with the given parameters.
*      		@mac_addr will be %NULL when requesting information for a group
*      		key. All pointers given to the @callback function need not be valid
*      		after it returns. This function should return an error if it is
*      		not possible to retrieve the key, -ENOENT if it doesn't exist.
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_get_key(struct wiphy *wiphy, struct net_device *netdev,u8 key_index,
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
		bool pairwise,
	#endif
	const u8 *mac_addr,void *cookie,void (*callback)(void* cookie, struct key_params*))
{

		WILC_Sint32 s32Error = WILC_SUCCESS;

		struct WILC_WFI_priv* priv;
		struct  key_params key_params;
		WILC_Uint32 i;
		priv= wiphy_priv(wiphy);


		#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
		if (!pairwise)
		#else
		if (!mac_addr || is_broadcast_ether_addr(mac_addr))
		#endif
		{
			PRINT_D(CFG80211_DBG,"Getting group key idx: %x\n",key_index);

			key_params.key=priv->wilc_gtk[key_index]->key;
			key_params.cipher=priv->wilc_gtk[key_index]->cipher;
			key_params.key_len=priv->wilc_gtk[key_index]->key_len;
			key_params.seq=priv->wilc_gtk[key_index]->seq;
			key_params.seq_len=priv->wilc_gtk[key_index]->seq_len;
			if(INFO)
			{
			for(i=0;i<key_params.key_len;i++)
				PRINT_INFO(CFG80211_DBG,"Retrieved key value %x\n",key_params.key[i]);
			}
		}
		else
		{
			PRINT_D(CFG80211_DBG,"Getting pairwise  key\n");

			key_params.key=priv->wilc_ptk[key_index]->key;
			key_params.cipher=priv->wilc_ptk[key_index]->cipher;
			key_params.key_len=priv->wilc_ptk[key_index]->key_len;
			key_params.seq=priv->wilc_ptk[key_index]->seq;
			key_params.seq_len=priv->wilc_ptk[key_index]->seq_len;
		}

		callback(cookie,&key_params);

		return s32Error;//priv->wilc_gtk->key_len ?0 : -ENOENT;
}

/**
*  @brief 	WILC_WFI_set_default_key
*  @details 	Set the default management frame key on an interface
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_set_default_key(struct wiphy *wiphy,struct net_device *netdev,u8 key_index
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,37)
		,bool unicast, bool multicast
	#endif
		)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	priv = wiphy_priv(wiphy);
	PRINT_D(CFG80211_DBG,"Setting default key with idx = %d \n", key_index);
	host_int_set_WEPDefaultKeyID(priv->hWILCWFIDrv,key_index);
	return s32Error;
}

/**
*  @brief 	WILC_WFI_dump_survey
*  @details 	Get site survey information
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_dump_survey(struct wiphy *wiphy, struct net_device *netdev,
	int idx, struct survey_info *info)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;


	if (idx != 0)
	{
		s32Error = -ENOENT;
		PRINT_ER("Error Idx value doesn't equal zero: Error(%d)\n",s32Error);

	}

	return s32Error;
}


/**
*  @brief 	WILC_WFI_get_station
*  @details 	Get station information for the station identified by @mac
*  @param[in]   NONE
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/

extern  uint32_t Statisitcs_totalAcks,Statisitcs_DroppedAcks;
static int WILC_WFI_get_station(struct wiphy *wiphy, struct net_device *dev,
	u8 *mac, struct station_info *sinfo)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	perInterface_wlan_t* nic;
	#ifdef WILC_AP_EXTERNAL_MLME
	WILC_Uint32 i =0;
	WILC_Uint32 associatedsta = 0;
	WILC_Uint32 inactive_time=0;
	#endif
	priv = wiphy_priv(wiphy);
	nic = netdev_priv(dev);

	#ifdef WILC_AP_EXTERNAL_MLME
	if(nic->iftype == AP_MODE || nic->iftype == GO_MODE)
	{
		PRINT_D(HOSTAPD_DBG,"Getting station parameters\n");

		PRINT_INFO(HOSTAPD_DBG, ": %x%x%x%x%x\n",mac[0],mac[1],mac[2],mac[3],mac[4]);

		for(i=0; i<NUM_STA_ASSOCIATED; i++)
		{

			if (!(memcmp(mac, priv->assoc_stainfo.au8Sta_AssociatedBss[i], ETH_ALEN)))
			{
			   associatedsta = i;
			   break;
			}

		}

		if(associatedsta == -1)
		{
			s32Error = -ENOENT;
			PRINT_ER("Station required is not associated : Error(%d)\n",s32Error);

			return s32Error;
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)	//0421
		sinfo->filled |= BIT(NL80211_STA_INFO_INACTIVE_TIME);
#else
		sinfo->filled |= STATION_INFO_INACTIVE_TIME;
#endif

		host_int_get_inactive_time(priv->hWILCWFIDrv, mac,&(inactive_time));
		sinfo->inactive_time = 1000 * inactive_time;
		PRINT_D(CFG80211_DBG,"Inactive time %d\n",sinfo->inactive_time);

	}
	#endif

	if(nic->iftype == STATION_MODE)
	{
		struct tstrStatistics strStatistics;
		host_int_get_statistics(priv->hWILCWFIDrv,&strStatistics);


	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)	//0421
		sinfo->filled |= BIT(NL80211_STA_INFO_SIGNAL) |
						BIT( NL80211_STA_INFO_RX_PACKETS) |
						BIT(NL80211_STA_INFO_TX_PACKETS) |
						BIT(NL80211_STA_INFO_TX_FAILED) |
						BIT(NL80211_STA_INFO_TX_BITRATE);
    #elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
		sinfo->filled |= STATION_INFO_SIGNAL | STATION_INFO_RX_PACKETS | STATION_INFO_TX_PACKETS
			| STATION_INFO_TX_FAILED | STATION_INFO_TX_BITRATE;
    #else
        sinfo->filled |= STATION_INFO_SIGNAL | STATION_INFO_RX_PACKETS | STATION_INFO_TX_PACKETS
            | STATION_INFO_TX_BITRATE;
    #endif

		sinfo->signal  		=  strStatistics.s8RSSI;
		sinfo->rx_packets   =  strStatistics.u32RxCount;
		sinfo->tx_packets   =  strStatistics.u32TxCount + strStatistics.u32TxFailureCount;
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
        sinfo->tx_failed	=  strStatistics.u32TxFailureCount;
    #endif
		sinfo->txrate.legacy = strStatistics.u8LinkSpeed*10;

#ifdef TCP_ENHANCEMENTS
		if((strStatistics.u8LinkSpeed > TCP_ACK_FILTER_LINK_SPEED_THRESH) && (strStatistics.u8LinkSpeed != DEFAULT_LINK_SPEED))
		{
			Enable_TCP_ACK_Filter(WILC_TRUE);
		}
		else if( strStatistics.u8LinkSpeed != DEFAULT_LINK_SPEED)
		{
			Enable_TCP_ACK_Filter(WILC_FALSE);
		}
#endif

    #if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
		PRINT_D(CORECONFIG_DBG, "*** stats[%d][%d][%d][%d][%d]\n",sinfo->signal,sinfo->rx_packets,sinfo->tx_packets,
			sinfo->tx_failed,sinfo->txrate.legacy);
    #else
        PRINT_D(CORECONFIG_DBG, "*** stats[%d][%d][%d][%d]\n",sinfo->signal,sinfo->rx_packets,sinfo->tx_packets,
            sinfo->txrate.legacy);
    #endif
	}
	return s32Error;
}


/**
*  @brief 	WILC_WFI_change_bss
*  @details 	Modify parameters for a given BSS.
*  @param[in]
*   -use_cts_prot: Whether to use CTS protection
*	    (0 = no, 1 = yes, -1 = do not change)
 *  -use_short_preamble: Whether the use of short preambles is allowed
 *	    (0 = no, 1 = yes, -1 = do not change)
 *  -use_short_slot_time: Whether the use of short slot time is allowed
 *	    (0 = no, 1 = yes, -1 = do not change)
 *  -basic_rates: basic rates in IEEE 802.11 format
 *	    (or NULL for no change)
 *  -basic_rates_len: number of basic rates
 *  -ap_isolate: do not forward packets between connected stations
 *  -ht_opmode: HT Operation mode
 * 	   (u16 = opmode, -1 = do not change)
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int  WILC_WFI_change_bss(struct wiphy *wiphy, struct net_device *dev,
	struct bss_parameters *params)
{
	PRINT_D(CFG80211_DBG,"Changing Bss parametrs\n");
	return 0;
}

/**
*  @brief 	WILC_WFI_auth
*  @details 	Request to authenticate with the specified peer
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_auth(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_auth_request *req)
{
	PRINT_D(CFG80211_DBG,"In Authentication Function\n");
	return 0;
}

/**
*  @brief 	WILC_WFI_assoc
*  @details 	Request to (re)associate with the specified peer
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_assoc(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_assoc_request *req)
{
	PRINT_D(CFG80211_DBG,"In Association Function\n");
	return 0;
}

/**
*  @brief 	WILC_WFI_deauth
*  @details 	Request to deauthenticate from the specified peer
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int  WILC_WFI_deauth(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_deauth_request *req,void *cookie)
{
	PRINT_D(CFG80211_DBG,"In De-authentication Function\n");
	return 0;
}

/**
*  @brief 	WILC_WFI_disassoc
*  @details 	Request to disassociate from the specified peer
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int  WILC_WFI_disassoc(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_disassoc_request *req,void *cookie)
{
	PRINT_D(CFG80211_DBG, "In Disassociation Function\n");
	return 0;
}

/**
*  @brief 	WILC_WFI_set_wiphy_params
*  @details 	Notify that wiphy parameters have changed;
*  @param[in]   Changed bitfield (see &enum wiphy_params_flags) describes which values
*      		have changed.
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct tstrCfgParamVal pstrCfgParamVal;
	struct WILC_WFI_priv* priv;

	priv = wiphy_priv(wiphy);
//	priv = netdev_priv(priv->wdev->netdev);

	pstrCfgParamVal.u32SetCfgFlag = 0;
	PRINT_D(CFG80211_DBG,"Setting Wiphy params \n");

	if(changed & WIPHY_PARAM_RETRY_SHORT)
	{
		PRINT_D(CFG80211_DBG,"Setting WIPHY_PARAM_RETRY_SHORT %d\n",
			priv->dev->ieee80211_ptr->wiphy->retry_short);
		pstrCfgParamVal.u32SetCfgFlag  |= RETRY_SHORT;
		pstrCfgParamVal.short_retry_limit = priv->dev->ieee80211_ptr->wiphy->retry_short;
	}
	if(changed & WIPHY_PARAM_RETRY_LONG)
	{

		PRINT_D(CFG80211_DBG,"Setting WIPHY_PARAM_RETRY_LONG %d\n",priv->dev->ieee80211_ptr->wiphy->retry_long);
		pstrCfgParamVal.u32SetCfgFlag |= RETRY_LONG;
		pstrCfgParamVal.long_retry_limit = priv->dev->ieee80211_ptr->wiphy->retry_long;

	}
	if(changed & WIPHY_PARAM_FRAG_THRESHOLD)
	{
		PRINT_D(CFG80211_DBG,"Setting WIPHY_PARAM_FRAG_THRESHOLD %d\n",priv->dev->ieee80211_ptr->wiphy->frag_threshold);
		pstrCfgParamVal.u32SetCfgFlag |= FRAG_THRESHOLD;
		pstrCfgParamVal.frag_threshold = priv->dev->ieee80211_ptr->wiphy->frag_threshold;

	}

	if(changed & WIPHY_PARAM_RTS_THRESHOLD)
	{
		PRINT_D(CFG80211_DBG, "Setting WIPHY_PARAM_RTS_THRESHOLD %d\n",priv->dev->ieee80211_ptr->wiphy->rts_threshold);

		pstrCfgParamVal.u32SetCfgFlag |= RTS_THRESHOLD;
		pstrCfgParamVal.rts_threshold = priv->dev->ieee80211_ptr->wiphy->rts_threshold;

	}

	PRINT_D(CFG80211_DBG,"Setting CFG params in the host interface\n");
	s32Error = hif_set_cfg(priv->hWILCWFIDrv,&pstrCfgParamVal);
	if(s32Error)
		PRINT_ER("Error in setting WIPHY PARAMS\n");


	return s32Error;
}
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
/**
*  @brief 	WILC_WFI_set_bitrate_mask
*  @details 	set the bitrate mask configuration
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_set_bitrate_mask(struct wiphy *wiphy,
	struct net_device *dev,const u8 *peer,
	const struct cfg80211_bitrate_mask *mask)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	//strCfgParamVal pstrCfgParamVal;
	//struct WILC_WFI_priv* priv;

	PRINT_D(CFG80211_DBG, "Setting Bitrate mask function\n");
#if 0
	priv = wiphy_priv(wiphy);
	//priv = netdev_priv(priv->wdev->netdev);

	pstrCfgParamVal.curr_tx_rate = mask->control[IEEE80211_BAND_2GHZ].legacy;

	PRINT_D(CFG80211_DBG, "Tx rate = %d\n",pstrCfgParamVal.curr_tx_rate);
	s32Error = hif_set_cfg(priv->hWILCWFIDrv,&pstrCfgParamVal);

	if(s32Error)
		PRINT_ER("Error in setting bitrate\n");
#endif
	return s32Error;

}

/**
*  @brief 	WILC_WFI_set_pmksa
*  @details 	Cache a PMKID for a BSSID. This is mostly useful for fullmac
*      		devices running firmwares capable of generating the (re) association
*      		RSN IE. It allows for faster roaming between WPA2 BSSIDs.
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_set_pmksa(struct wiphy *wiphy, struct net_device *netdev,
	struct cfg80211_pmksa *pmksa)
{
	WILC_Uint32 i;
	WILC_Sint32 s32Error = WILC_SUCCESS;
	WILC_Uint8 flag = 0;

	struct WILC_WFI_priv* priv = wiphy_priv(wiphy);

	PRINT_D(CFG80211_DBG, "Setting PMKSA\n");


	for (i = 0; i < priv->pmkid_list.numpmkid; i++)
	{
		if (!WILC_memcmp(pmksa->bssid,priv->pmkid_list.pmkidlist[i].bssid,
				ETH_ALEN))
		{
			/*If bssid already exists and pmkid value needs to reset*/
			flag = PMKID_FOUND;
			PRINT_D(CFG80211_DBG, "PMKID already exists\n");
			break;
		}
	}
	if (i < WILC_MAX_NUM_PMKIDS) {
		PRINT_D(CFG80211_DBG, "Setting PMKID in private structure\n");
		WILC_memcpy(priv->pmkid_list.pmkidlist[i].bssid, pmksa->bssid,
				ETH_ALEN);
		WILC_memcpy(priv->pmkid_list.pmkidlist[i].pmkid, pmksa->pmkid,
				PMKID_LEN);
		if(!(flag == PMKID_FOUND))
			priv->pmkid_list.numpmkid++;
	}
	else
	{
		PRINT_ER("Invalid PMKID index\n");
		s32Error = -EINVAL;
	}

	if(!s32Error)
	{
		PRINT_D(CFG80211_DBG, "Setting pmkid in the host interface\n");
		s32Error = host_int_set_pmkid_info(priv->hWILCWFIDrv, &priv->pmkid_list);
	}
	return s32Error;
}

/**
*  @brief 	WILC_WFI_del_pmksa
*  @details 	Delete a cached PMKID.
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_del_pmksa(struct wiphy *wiphy, struct net_device *netdev,
	struct cfg80211_pmksa *pmksa)
{

	WILC_Uint32 i;
	WILC_Uint8 flag = 0;
	WILC_Sint32 s32Error = WILC_SUCCESS;

	struct WILC_WFI_priv* priv = wiphy_priv(wiphy);
	//priv = netdev_priv(priv->wdev->netdev);

	PRINT_D(CFG80211_DBG, "Deleting PMKSA keys\n");

	for (i = 0; i < priv->pmkid_list.numpmkid; i++)
	{
		if (!WILC_memcmp(pmksa->bssid,priv->pmkid_list.pmkidlist[i].bssid,
					ETH_ALEN))
		{
			/*If bssid is found, reset the values*/
			PRINT_D(CFG80211_DBG, "Reseting PMKID values\n");
			WILC_memset(&priv->pmkid_list.pmkidlist[i], 0, sizeof(struct tstrHostIFpmkid));
			flag = PMKID_FOUND;
			break;
		}
	}

	if(i < priv->pmkid_list.numpmkid && priv->pmkid_list.numpmkid > 0)
	{
		for (; i < (priv->pmkid_list.numpmkid- 1); i++) {
				WILC_memcpy(priv->pmkid_list.pmkidlist[i].bssid,
					priv->pmkid_list.pmkidlist[i+1].bssid,
					ETH_ALEN);
				WILC_memcpy(priv->pmkid_list.pmkidlist[i].pmkid,
					priv->pmkid_list.pmkidlist[i].pmkid,
					PMKID_LEN);
			}
		priv->pmkid_list.numpmkid--;
	}
	else {
			s32Error = -EINVAL;
		}

	return s32Error;
}

/**
*  @brief 	WILC_WFI_flush_pmksa
*  @details 	Flush all cached PMKIDs.
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int  WILC_WFI_flush_pmksa(struct wiphy *wiphy, struct net_device *netdev)
{
	struct WILC_WFI_priv* priv = wiphy_priv(wiphy);
	//priv = netdev_priv(priv->wdev->netdev);

	PRINT_D(CFG80211_DBG,  "Flushing  PMKID key values\n");

	/*Get cashed Pmkids and set all with zeros*/
	WILC_memset(&priv->pmkid_list, 0, sizeof(struct tstrHostIFpmkidAttr) );

	return 0;
}
#endif

#ifdef WILC_P2P

/**
*  @brief 	WILC_WFI_CfgParseRxAction
*  @details Function parses the received  frames and modifies the following attributes:
*                -GO Intent
*		    -Channel list
*		    -Operating Channel
*
*  @param[in] u8* Buffer, u32 length
*  @return 	NONE.
*  @author	mdaftedar
*  @date	12 DEC 2012
*  @version
*/

void WILC_WFI_CfgParseRxAction(WILC_Uint8 * buf,WILC_Uint32 len)
{
	WILC_Uint32 index=0;
	WILC_Uint32 i=0,j=0;

	/*BugID_5460*/
	#ifdef USE_SUPPLICANT_GO_INTENT
	WILC_Uint8 intent;
	WILC_Uint8 tie_breaker;
	WILC_Bool is_wilc_go = WILC_TRUE;
	#endif
	WILC_Uint8 op_channel_attr_index = 0;
	WILC_Uint8 channel_list_attr_index = 0;

	while(index < len)
	{
		if(buf[index] == GO_INTENT_ATTR_ID)
		{
			#ifdef USE_SUPPLICANT_GO_INTENT
			/*BugID_5460*/
			/*Case 1: If we are going to be p2p client, no need to modify channels attributes*/
			/*In negotiation frames, go intent attr value determines who will be GO*/
			intent = GET_GO_INTENT(buf[index+3]);
			tie_breaker = GET_TIE_BREAKER(buf[index+3]);
			if(intent > SUPPLICANT_GO_INTENT
				|| (intent == SUPPLICANT_GO_INTENT && tie_breaker == 1))
			{
				PRINT_D(GENERIC_DBG, "WILC will be client (intent %d tie breaker %d)\n", intent, tie_breaker);
				is_wilc_go = WILC_FALSE;
			}
			else
			{
				PRINT_D(GENERIC_DBG, "WILC will be GO (intent %d tie breaker %d)\n", intent, tie_breaker);
				is_wilc_go = WILC_TRUE;
			}

			#else	//USE_SUPPLICANT_GO_INTENT
			#ifdef FORCE_P2P_CLIENT
			buf[index+3] =(buf[index+3]  & 0x01) | (0x0f << 1);
			#else
			buf[index+3] =(buf[index+3]  & 0x01) | (0x00 << 1);
			#endif
			#endif	//USE_SUPPLICANT_GO_INTENT
		}

		#ifdef USE_SUPPLICANT_GO_INTENT
		/*Case 2: If group bssid attribute is present, no need to modify channels attributes*/
		/*In invitation req and rsp, group bssid attr presence determines who will be GO*/
		if(buf[index] == GROUP_BSSID_ATTR_ID)
		{
			PRINT_D(GENERIC_DBG, "Group BSSID: %2x:%2x:%2x\n", buf[index+3]
															, buf[index+4]
															, buf[index+5]);
			is_wilc_go = WILC_FALSE;
		}
		#endif	//USE_SUPPLICANT_GO_INTENT

		if(buf[index] ==  CHANLIST_ATTR_ID)
		{
			channel_list_attr_index = index;
		}
		else if(buf[index] ==  OPERCHAN_ATTR_ID)
		{
			op_channel_attr_index = index;
		}
		index+=buf[index+1]+3; //ID,Length byte
	}

	#ifdef USE_SUPPLICANT_GO_INTENT
	if(u8WLANChannel != INVALID_CHANNEL && is_wilc_go)
	#else
	if(u8WLANChannel != INVALID_CHANNEL)
	#endif
	{
		/*Modify channel list attribute*/
		if(channel_list_attr_index)
		{
			PRINT_D(GENERIC_DBG, "Modify channel list attribute\n");
			for(i=channel_list_attr_index+3;i<((channel_list_attr_index+3)+buf[channel_list_attr_index+1]);i++)
			{
				if(buf[i] == 0x51)
				{
					for(j=i+2;j<((i+2)+buf[i+1]);j++)
					{
						buf[j] = u8WLANChannel;
					}
					break;
				}
			}
		}
		/*Modify operating channel attribute*/
		if(op_channel_attr_index)
		{
			PRINT_D(GENERIC_DBG, "Modify operating channel attribute\n");
			buf[op_channel_attr_index+6]=0x51;
			buf[op_channel_attr_index+7]=u8WLANChannel;
		}
	}
}

/**
*  @brief 	WILC_WFI_CfgParseTxAction
*  @details Function parses the transmitted  action frames and modifies the
*               GO Intent attribute
*  @param[in] u8* Buffer, u32 length, bool bOperChan, u8 iftype
*  @return 	NONE.
*  @author	mdaftedar
*  @date	12 DEC 2012
*  @version
*/
void WILC_WFI_CfgParseTxAction(WILC_Uint8 * buf,WILC_Uint32 len,WILC_Bool bOperChan, WILC_Uint8 iftype)
{
	WILC_Uint32 index=0;
	WILC_Uint32 i=0,j=0;

	WILC_Uint8 op_channel_attr_index = 0;
	WILC_Uint8 channel_list_attr_index = 0;
	#ifdef USE_SUPPLICANT_GO_INTENT
	WILC_Bool is_wilc_go = WILC_FALSE;

	/*BugID_5460*/
	/*Case 1: If we are already p2p client, no need to modify channels attributes*/
	/*This to handle the case of inviting a p2p peer to join an existing group which we are a member in*/
	if(iftype == CLIENT_MODE)
		return;
	#endif

	while(index < len)
	{
		#ifdef USE_SUPPLICANT_GO_INTENT
		/*Case 2: If group bssid attribute is present, no need to modify channels attributes*/
		/*In invitation req and rsp, group bssid attr presence determines who will be GO*/
		/*Note: If we are already p2p client, group bssid attr may also be present (handled in Case 1)*/
		if(buf[index] == GROUP_BSSID_ATTR_ID)
		{
			PRINT_D(GENERIC_DBG, "Group BSSID: %2x:%2x:%2x\n", buf[index+3]
															, buf[index+4]
															, buf[index+5]);
			is_wilc_go = WILC_TRUE;
		}

		#else	//USE_SUPPLICANT_GO_INTENT
		 if(buf[index] == GO_INTENT_ATTR_ID)
		{
			#ifdef FORCE_P2P_CLIENT
			buf[index+3] =(buf[index+3]  & 0x01) | (0x00 << 1);
			#else
			buf[index+3] =(buf[index+3]  & 0x01) | (0x0f << 1);
			#endif

			break;
		}
		#endif

		 if(buf[index] ==  CHANLIST_ATTR_ID)
		{
			channel_list_attr_index = index;
		}
		else if(buf[index] ==  OPERCHAN_ATTR_ID)
		{
			op_channel_attr_index = index;
		}
		index+=buf[index+1]+3; //ID,Length byte
	}

	#ifdef USE_SUPPLICANT_GO_INTENT
	/*No need to check bOperChan since only transmitted invitation frames are parsed*/
	if(u8WLANChannel != INVALID_CHANNEL && is_wilc_go)
	#else
	if(u8WLANChannel != INVALID_CHANNEL && bOperChan)
	#endif
	{
		/*Modify channel list attribute*/
		if(channel_list_attr_index)
		{
			PRINT_D(GENERIC_DBG, "Modify channel list attribute\n");
			for(i=channel_list_attr_index+3;i<((channel_list_attr_index+3)+buf[channel_list_attr_index+1]);i++)
			{
				if(buf[i] == 0x51)
				{
					for(j=i+2;j<((i+2)+buf[i+1]);j++)
					{
						buf[j] = u8WLANChannel;
					}
					break;
				}
			}
		}
		/*Modify operating channel attribute*/
		if(op_channel_attr_index)
		{
			PRINT_D(GENERIC_DBG, "Modify operating channel attribute\n");
			buf[op_channel_attr_index+6]=0x51;
			buf[op_channel_attr_index+7]=u8WLANChannel;
		}
	}
}

/*  @brief 			 WILC_WFI_p2p_rx
*  @details
*  @param[in]

*  @return 		None
*  @author		Mai Daftedar
*  @date			2 JUN 2013
*  @version		1.0
*/

void WILC_WFI_p2p_rx (struct net_device *dev,uint8_t *buff, uint32_t size)
{

	struct WILC_WFI_priv* priv;
	WILC_Uint32 header,pkt_offset;
	struct WILC_WFIDrv * pstrWFIDrv;
	WILC_Uint32 i=0;
	WILC_Sint32 s32Freq;
	priv= wiphy_priv(dev->ieee80211_ptr->wiphy);
	pstrWFIDrv = (struct WILC_WFIDrv *)priv->hWILCWFIDrv;

	//Get WILC header
	WILC_memcpy(&header, (buff-HOST_HDR_OFFSET), HOST_HDR_OFFSET);

	//The packet offset field conain info about what type of managment frame
	// we are dealing with and ack status
	pkt_offset = GET_PKT_OFFSET(header);

	if(pkt_offset & IS_MANAGMEMENT_CALLBACK)
	{
		if(buff[FRAME_TYPE_ID]==IEEE80211_STYPE_PROBE_RESP)
		{
			PRINT_D(GENERIC_DBG,"Probe response ACK\n");
			#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0))
			cfg80211_mgmt_tx_status(dev,priv->u64tx_cookie,buff,size,true,GFP_KERNEL);
			#else
			cfg80211_mgmt_tx_status(priv->wdev,priv->u64tx_cookie,buff,size,true,GFP_KERNEL);
			#endif
			return;
		}
		else
		{
        	if(pkt_offset & IS_MGMT_STATUS_SUCCES)
        	{
				PRINT_D(GENERIC_DBG,"Success Ack - Action frame category: %x Action Subtype: %d Dialog T: %x OR %x\n",buff[ACTION_CAT_ID], buff[ACTION_SUBTYPE_ID],
					buff[ACTION_SUBTYPE_ID+1], buff[P2P_PUB_ACTION_SUBTYPE+1]);
				#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0))
        		cfg80211_mgmt_tx_status(dev,priv->u64tx_cookie,buff,size,true,GFP_KERNEL);
				#else
				cfg80211_mgmt_tx_status(priv->wdev,priv->u64tx_cookie,buff,size,true,GFP_KERNEL);
				#endif
        	}
			else
			{
				PRINT_D(GENERIC_DBG,"Fail Ack - Action frame category: %x Action Subtype: %d Dialog T: %x OR %x\n",buff[ACTION_CAT_ID], buff[ACTION_SUBTYPE_ID],
					buff[ACTION_SUBTYPE_ID+1], buff[P2P_PUB_ACTION_SUBTYPE+1]);
				#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0))
				cfg80211_mgmt_tx_status(dev,priv->u64tx_cookie,buff,size,false,GFP_KERNEL);
				#else
				cfg80211_mgmt_tx_status(priv->wdev,priv->u64tx_cookie,buff,size,false,GFP_KERNEL);
				#endif
			}
			return;
	 	 }
	}
	else
	{

		 PRINT_D(GENERIC_DBG,"Rx Frame Type:%x\n",buff[FRAME_TYPE_ID]);

		/*BugID_5442*/
		/*Upper layer is informed that the frame is received on this freq*/
		#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,38)
			s32Freq = ieee80211_channel_to_frequency(u8CurrChannel, IEEE80211_BAND_2GHZ);
		#else
			s32Freq = ieee80211_channel_to_frequency(u8CurrChannel);
		#endif

		if (ieee80211_is_action(buff[FRAME_TYPE_ID]) )
		{
			PRINT_D(GENERIC_DBG,"Rx Action Frame Type: %x %x\n", buff[ACTION_SUBTYPE_ID] ,buff[P2P_PUB_ACTION_SUBTYPE]);

			if(priv->bCfgScanning == WILC_TRUE && jiffies >= pstrWFIDrv->p2p_mgmt_timeout)
			{
				PRINT_D(GENERIC_DBG,"Receiving action frames from wrong channels\n");
				return;
			}
			if(buff[ACTION_CAT_ID] == PUB_ACTION_ATTR_ID)
			{

				switch(buff[ACTION_SUBTYPE_ID])
				{
					case GAS_INTIAL_REQ:
					{
						PRINT_D(GENERIC_DBG,"GAS INITIAL REQ %x\n",buff[ACTION_SUBTYPE_ID]);
						break;
					}
					case GAS_INTIAL_RSP:
					{
						PRINT_D(GENERIC_DBG,"GAS INITIAL RSP %x\n",buff[ACTION_SUBTYPE_ID]);
						break;
					}
					case PUBLIC_ACT_VENDORSPEC:
					{
						/*Now we have a public action vendor specific action frame, check if its a p2p public action frame
						based on the standard its should have the p2p_oui attribute with the following values 50 6f 9A 09*/
						if(!WILC_memcmp(u8P2P_oui,&buff[ACTION_SUBTYPE_ID+1],4))
						{
							if ((buff[P2P_PUB_ACTION_SUBTYPE] == GO_NEG_REQ ||  buff[P2P_PUB_ACTION_SUBTYPE] == GO_NEG_RSP))
							{
								if(!bWilc_ie)
								{
									for(i=P2P_PUB_ACTION_SUBTYPE;i<size;i++)
									{
										if(!WILC_memcmp(u8P2P_vendorspec,&buff[i],6))
										{
											u8P2Precvrandom = buff[i+6];
											bWilc_ie=WILC_TRUE;
											PRINT_D(GENERIC_DBG,"WILC Vendor specific IE:%02x\n",u8P2Precvrandom);
											break;
										}
									}
								}
							}
							if(u8P2Plocalrandom > u8P2Precvrandom)
							{
								if ( ( buff[P2P_PUB_ACTION_SUBTYPE] == GO_NEG_REQ ||  buff[P2P_PUB_ACTION_SUBTYPE] == GO_NEG_RSP
									|| buff[P2P_PUB_ACTION_SUBTYPE] == P2P_INV_REQ ||  buff[P2P_PUB_ACTION_SUBTYPE] == P2P_INV_RSP))
								{
									for(i=P2P_PUB_ACTION_SUBTYPE+2;i<size;i++)
									{
								       	if(buff[i]== P2PELEM_ATTR_ID && !(WILC_memcmp(u8P2P_oui,&buff[i+2],4)) )
										{
											WILC_WFI_CfgParseRxAction(&buff[i+6],size-(i+6));
											break;
										}
									}
								}
							}
							else
								PRINT_D(GENERIC_DBG,"PEER WILL BE GO LocaRand=%02x RecvRand %02x\n",u8P2Plocalrandom,u8P2Precvrandom);
						}


						if( (buff[P2P_PUB_ACTION_SUBTYPE] == GO_NEG_REQ ||  buff[P2P_PUB_ACTION_SUBTYPE] == GO_NEG_RSP ) && ( bWilc_ie))
						{
							PRINT_D(GENERIC_DBG,"Sending P2P to host without extra elemnt\n");
							//extra attribute for sig_dbm: signal strength in mBm, or 0 if unknown
							#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
							cfg80211_rx_mgmt(priv->wdev,s32Freq, 0, buff,size-7,0);
							#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0))
							cfg80211_rx_mgmt(priv->wdev,s32Freq, 0, buff,size-7,0,GFP_ATOMIC);
							#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0))
							cfg80211_rx_mgmt(priv->wdev,s32Freq, 0, buff,size-7,GFP_ATOMIC);
							#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))
							cfg80211_rx_mgmt(dev, s32Freq, 0, buff,size-7,GFP_ATOMIC);	// rachel
							#elif (LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
							cfg80211_rx_mgmt(dev,s32Freq,buff,size-7,GFP_ATOMIC);
							#endif

							return;
						}
						break;
					}
					default:
					{
						PRINT_D(GENERIC_DBG,"NOT HANDLED PUBLIC ACTION FRAME TYPE:%x\n",buff[ACTION_SUBTYPE_ID]);
						break;
					}
				}
			}
		}

		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18 ,0))
		cfg80211_rx_mgmt(priv->wdev,s32Freq, 0, buff, size, 0);
		#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0))
		cfg80211_rx_mgmt(priv->wdev,s32Freq, 0, buff, size, 0, GFP_ATOMIC);
		#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0))
		cfg80211_rx_mgmt(priv->wdev,s32Freq, 0, buff,size,GFP_ATOMIC);
		#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))
		cfg80211_rx_mgmt(dev,s32Freq, 0, buff,size,GFP_ATOMIC);
		#elif (LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
		cfg80211_rx_mgmt(dev,s32Freq,buff,size,GFP_ATOMIC);
		#endif
	}
}

/**
*  @brief 			WILC_WFI_mgmt_tx_complete
*  @details 		Returns result of writing mgmt frame to VMM (Tx buffers are freed here)
*  @param[in]   	priv
				transmitting status
*  @return 		None
*  @author		Amr Abdelmoghny
*  @date			20 MAY 2013
*  @version		1.0
*/
static void WILC_WFI_mgmt_tx_complete(void* priv, int status)
{
	struct p2p_mgmt_data* pv_data = (struct p2p_mgmt_data*)priv;


	kfree(pv_data->buff);
	kfree(pv_data);
}

/**
* @brief 		WILC_WFI_RemainOnChannelReady
*  @details 	Callback function, called from handle_remain_on_channel on being ready on channel
*  @param
*  @return 	none
*  @author	Amr abdelmoghny
*  @date		9 JUNE 2013
*  @version
*/

static void WILC_WFI_RemainOnChannelReady(void* pUserVoid)
{
	struct WILC_WFI_priv* priv;
	priv = (struct WILC_WFI_priv*)pUserVoid;

	PRINT_D(HOSTINF_DBG, "Remain on channel ready \n");

	priv->bInP2PlistenState = WILC_TRUE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	cfg80211_ready_on_channel(priv->wdev,
							priv->strRemainOnChanParams.u64ListenCookie,
							priv->strRemainOnChanParams.pstrListenChan,
							priv->strRemainOnChanParams.u32ListenDuration,
							GFP_KERNEL);
#else
	cfg80211_ready_on_channel(priv->dev,
							priv->strRemainOnChanParams.u64ListenCookie,
							priv->strRemainOnChanParams.pstrListenChan,
							priv->strRemainOnChanParams.tenuChannelType,
							priv->strRemainOnChanParams.u32ListenDuration,
							GFP_KERNEL);
#endif
}

/**
* @brief 		WILC_WFI_RemainOnChannelExpired
*  @details 	Callback function, called on expiration of remain-on-channel duration
*  @param
*  @return 	none
*  @author	Amr abdelmoghny
*  @date		15 MAY 2013
*  @version
*/

static void WILC_WFI_RemainOnChannelExpired(void* pUserVoid, WILC_Uint32 u32SessionID)
{
	struct WILC_WFI_priv* priv;
	priv = (struct WILC_WFI_priv*)pUserVoid;

	/*BugID_5477*/
	if(u32SessionID == priv->strRemainOnChanParams.u32ListenSessionID)
	{
		PRINT_D(GENERIC_DBG, "Remain on channel expired \n");

		priv->bInP2PlistenState = WILC_FALSE;

		/*Inform wpas of remain-on-channel expiration*/
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		cfg80211_remain_on_channel_expired(priv->wdev,
			priv->strRemainOnChanParams.u64ListenCookie,
			priv->strRemainOnChanParams.pstrListenChan ,
			GFP_KERNEL);
	#else
		cfg80211_remain_on_channel_expired(priv->dev,
			priv->strRemainOnChanParams.u64ListenCookie,
			priv->strRemainOnChanParams.pstrListenChan ,
			priv->strRemainOnChanParams.tenuChannelType,
			GFP_KERNEL);
	#endif
	}
	else
	{
		PRINT_D(GENERIC_DBG, "Received ID 0x%x Expected ID 0x%x (No match)\n", u32SessionID
											, priv->strRemainOnChanParams.u32ListenSessionID);
	}
}


/**
*  @brief 	WILC_WFI_remain_on_channel
*  @details 	Request the driver to remain awake on the specified
*      		channel for the specified duration to complete an off-channel
*      		operation (e.g., public action frame exchange). When the driver is
*      		ready on the requested channel, it must indicate this with an event
*      		notification by calling cfg80211_ready_on_channel().
*  @param[in]
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int  WILC_WFI_remain_on_channel(struct wiphy *wiphy,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0))
        struct wireless_dev *wdev,
#else
        struct net_device *dev,
#endif
	struct ieee80211_channel *chan,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0))
	enum nl80211_channel_type channel_type,
#endif
	unsigned int duration,u64 *cookie)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	priv = wiphy_priv(wiphy);

	PRINT_D(GENERIC_DBG, "Remaining on channel %d\n",chan->hw_value);

	/*BugID_4800: if in AP mode, return.*/
	/*This check is to handle the situation when user*/
	/*requests "create group" during a running scan*/

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0))
	if(wdev->iftype == NL80211_IFTYPE_AP)
	{
		PRINT_D(GENERIC_DBG,"Required remain-on-channel while in AP mode");
		return s32Error;
	}
#else
	if(dev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP)
	{
		PRINT_D(GENERIC_DBG,"Required remain-on-channel while in AP mode");
		return s32Error;
	}
#endif

	u8CurrChannel = chan->hw_value;

	/*Setting params needed by WILC_WFI_RemainOnChannelExpired()*/
	priv->strRemainOnChanParams.pstrListenChan = chan;
	priv->strRemainOnChanParams.u64ListenCookie = *cookie;
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0))
	priv->strRemainOnChanParams.tenuChannelType = channel_type;
	#endif
	priv->strRemainOnChanParams.u32ListenDuration = duration;
	priv->strRemainOnChanParams.u32ListenSessionID++;

	s32Error = host_int_remain_on_channel(priv->hWILCWFIDrv
										, priv->strRemainOnChanParams.u32ListenSessionID
										, duration
										, chan->hw_value
										, WILC_WFI_RemainOnChannelExpired
										, WILC_WFI_RemainOnChannelReady
										, (void *)priv);

	return s32Error;
}

/**
*  @brief 	WILC_WFI_cancel_remain_on_channel
*  @details 	Cancel an on-going remain-on-channel operation.
*      		This allows the operation to be terminated prior to timeout based on
*      		the duration value.
*  @param[in]   struct wiphy *wiphy,
*  @param[in] 	struct net_device *dev
*  @param[in] 	u64 cookie,
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int   WILC_WFI_cancel_remain_on_channel(struct wiphy *wiphy,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0))
	struct wireless_dev *wdev,
#else
	struct net_device *dev,
#endif
	u64 cookie)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	priv = wiphy_priv(wiphy);

	PRINT_D(CFG80211_DBG, "Cancel remain on channel\n");

	s32Error = host_int_ListenStateExpired(priv->hWILCWFIDrv, priv->strRemainOnChanParams.u32ListenSessionID);
	return s32Error;
}
/**
*  @brief 	 WILC_WFI_add_wilcvendorspec
*  @details 	Adding WILC information elemet to allow two WILC devices to
				identify each other and connect
*  @param[in]   WILC_Uint8 * buf
*  @return 	void
*  @author	mdaftedar
*  @date	01 JAN 2014
*  @version	1.0
*/
void WILC_WFI_add_wilcvendorspec(WILC_Uint8 * buff)
{
	WILC_memcpy(buff,u8P2P_vendorspec,sizeof(u8P2P_vendorspec));
}
/**
*  @brief 	WILC_WFI_mgmt_tx_frame
*  @details
*
*  @param[in]
*  @return 	NONE.
*  @author	mdaftedar
*  @date	01 JUL 2012
*  @version
*/
extern linux_wlan_t* g_linux_wlan;

int WILC_WFI_mgmt_tx(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
			struct wireless_dev *wdev,
			struct cfg80211_mgmt_tx_params *params,
			u64 *cookie)
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
			struct wireless_dev *wdev,
			struct ieee80211_channel *chan,
			bool offchan,
			unsigned int wait,
			const u8 *buf,
			size_t len,
			bool no_cck,
			bool dont_wait_for_ack, u64 *cookie)
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
			struct wireless_dev *wdev,
			struct ieee80211_channel *chan, bool offchan,
			enum nl80211_channel_type channel_type,
			bool channel_type valid,
			unsigned int wait, const u8 *buf,
			size_t len, bool no_cck,
			bool dont_wait_for_ack, u64 *cookie)
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
			struct net_device *dev,
			struct ieee80211_channel *chan, bool offchan,
			enum nl80211_channel_type channel_type,
			bool channel_type_valid,
			unsigned int wait, const u8 *buf,
			size_t len, bool no_cck,
			bool dont_wait_for_ack, u64 *cookie)
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
			struct net_device *dev,
			struct ieee80211_channel *chan, bool offchan,
			enum nl80211_channel_type channel_type,
			bool channel_type_valid,
			unsigned int wait, const u8 *buf,
			size_t len, bool no_cck, u64 *cookie)
#else
			struct net_device *dev,
			struct ieee80211_channel *chan, bool offchan,
			enum nl80211_channel_type channel_type,
			bool channel_type_valid,
			unsigned int wait, const u8 *buf,
			size_t len, u64 *cookie)
#endif
{
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	struct ieee80211_channel *chan = params->chan;
	unsigned int wait = params->wait;
	const u8 *buf = params->buf;
	size_t len = params->len;
	#endif
	const struct ieee80211_mgmt *mgmt;
	struct p2p_mgmt_data *mgmt_tx;
	struct WILC_WFI_priv* priv;
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFIDrv * pstrWFIDrv;
	WILC_Uint32 i;
	perInterface_wlan_t* nic;
	WILC_Uint32 buf_len = len + sizeof(u8P2P_vendorspec) + sizeof(u8P2Plocalrandom);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0))
	nic = netdev_priv(wdev->netdev);
#else
	nic = netdev_priv(dev);
#endif
	priv = wiphy_priv(wiphy);
	pstrWFIDrv = (struct WILC_WFIDrv *)priv->hWILCWFIDrv;

	*cookie = (unsigned long)buf;
	priv->u64tx_cookie = *cookie;
	mgmt = (const struct ieee80211_mgmt *) buf;

	if (ieee80211_is_mgmt(mgmt->frame_control))
	{

		/*mgmt frame allocation*/
		mgmt_tx = (struct p2p_mgmt_data*)WILC_MALLOC(sizeof(struct p2p_mgmt_data));
		if(mgmt_tx == NULL){
			PRINT_ER("Failed to allocate memory for mgmt_tx structure\n");
	     		return WILC_FAIL;
		}
		mgmt_tx->buff= (char*)WILC_MALLOC(buf_len);
		if(mgmt_tx->buff == NULL)
		{
			PRINT_ER("Failed to allocate memory for mgmt_tx buff\n");
	     		return WILC_FAIL;
		}
		WILC_memcpy(mgmt_tx->buff,buf,len);
		mgmt_tx->size=len;


		if (ieee80211_is_probe_resp(mgmt->frame_control))
		{
			PRINT_D(GENERIC_DBG,"TX: Probe Response\n");
			PRINT_D(GENERIC_DBG,"Setting channel: %d\n",chan->hw_value);
			host_int_set_mac_chnl_num(priv->hWILCWFIDrv, chan->hw_value);
			/*Save the current channel after we tune to it*/
			u8CurrChannel = chan->hw_value;
		}
		else if (ieee80211_is_action(mgmt->frame_control))
		{
			PRINT_D(GENERIC_DBG,"ACTION FRAME:%x\n",(WILC_Uint16)mgmt->frame_control);


			/*BugID_4847*/
			if(buf[ACTION_CAT_ID] == PUB_ACTION_ATTR_ID)
			{
				/*BugID_4847*/
				/*Only set the channel, if not a negotiation confirmation frame
				(If Negotiation confirmation frame, force it
				to be transmitted on the same negotiation channel)*/

				if(buf[ACTION_SUBTYPE_ID] != PUBLIC_ACT_VENDORSPEC ||
						buf[P2P_PUB_ACTION_SUBTYPE] !=GO_NEG_CONF)
				{
						PRINT_D(GENERIC_DBG,"Setting channel: %d\n",chan->hw_value);
						host_int_set_mac_chnl_num(priv->hWILCWFIDrv, chan->hw_value);
						/*Save the current channel after we tune to it*/
						u8CurrChannel = chan->hw_value;
				}
				switch(buf[ACTION_SUBTYPE_ID])
				{
					case GAS_INTIAL_REQ:
					{
						PRINT_D(GENERIC_DBG,"GAS INITIAL REQ %x\n",buf[ACTION_SUBTYPE_ID]);
						break;
					}
					case GAS_INTIAL_RSP:
					{
						PRINT_D(GENERIC_DBG,"GAS INITIAL RSP %x\n",buf[ACTION_SUBTYPE_ID]);
						break;
					}
					case PUBLIC_ACT_VENDORSPEC:
					{
						/*Now we have a public action vendor specific action frame, check if its a p2p public action frame
						based on the standard its should have the p2p_oui attribute with the following values 50 6f 9A 09*/
						if(!WILC_memcmp(u8P2P_oui,&buf[ACTION_SUBTYPE_ID+1],4))
						{
							/*For the connection of two WILC's connection generate a rand number to determine who will be a GO*/
							if ( ( buf[P2P_PUB_ACTION_SUBTYPE] == GO_NEG_REQ ||  buf[P2P_PUB_ACTION_SUBTYPE] == GO_NEG_RSP))
							{
								if(u8P2Plocalrandom == 1 && u8P2Precvrandom<u8P2Plocalrandom)
								{
									get_random_bytes(&u8P2Plocalrandom, 1);
									/*Increment the number to prevent if its 0*/
									u8P2Plocalrandom++;
								}
							}

							if ( ( buf[P2P_PUB_ACTION_SUBTYPE] == GO_NEG_REQ ||  buf[P2P_PUB_ACTION_SUBTYPE] == GO_NEG_RSP
									|| buf[P2P_PUB_ACTION_SUBTYPE] == P2P_INV_REQ ||  buf[P2P_PUB_ACTION_SUBTYPE] == P2P_INV_RSP))
							{
								if(u8P2Plocalrandom > u8P2Precvrandom)
								{
									PRINT_D(GENERIC_DBG,"LOCAL WILL BE GO LocaRand=%02x RecvRand %02x\n",u8P2Plocalrandom,u8P2Precvrandom);

									/*Search for the p2p information information element , after the Public action subtype theres a byte for teh dialog token, skip that*/
									for(i=P2P_PUB_ACTION_SUBTYPE+2;i<len;i++)
									{
								       	if(buf[i]== P2PELEM_ATTR_ID && !(WILC_memcmp(u8P2P_oui,&buf[i+2],4)) )
										{
											if(buf[P2P_PUB_ACTION_SUBTYPE]==P2P_INV_REQ || buf[P2P_PUB_ACTION_SUBTYPE]==P2P_INV_RSP)
												WILC_WFI_CfgParseTxAction(&mgmt_tx->buff[i+6],len-(i+6),WILC_TRUE, nic->iftype);

											/*BugID_5460*/
											/*If using supplicant go intent, no need at all*/
											/*to parse transmitted negotiation frames*/
											#ifndef USE_SUPPLICANT_GO_INTENT
											else
												WILC_WFI_CfgParseTxAction(&mgmt_tx->buff[i+6],len-(i+6),WILC_FALSE, nic->iftype);
											#endif
											break;
										}
									}

									if(buf[P2P_PUB_ACTION_SUBTYPE] !=P2P_INV_REQ && buf[P2P_PUB_ACTION_SUBTYPE] !=P2P_INV_RSP )
									{
										WILC_WFI_add_wilcvendorspec(&mgmt_tx->buff[len]);
										mgmt_tx->buff[len + sizeof(u8P2P_vendorspec)] = u8P2Plocalrandom;
										mgmt_tx->size=buf_len;
									}
								}
								else
									PRINT_D(GENERIC_DBG,"PEER WILL BE GO LocaRand=%02x RecvRand %02x\n",u8P2Plocalrandom,u8P2Precvrandom);
							}

						}
						else
						{
							PRINT_D(GENERIC_DBG,"Not a P2P public action frame\n");
						}

						break;
					}
					default:
					{
						PRINT_D(GENERIC_DBG,"NOT HANDLED PUBLIC ACTION FRAME TYPE:%x\n",buf[ACTION_SUBTYPE_ID]);
						break;
					}
				}

				}

		PRINT_D(GENERIC_DBG,"TX: ACTION FRAME Type:%x : Chan:%d\n",buf[ACTION_SUBTYPE_ID], chan->hw_value);
		pstrWFIDrv->p2p_mgmt_timeout = (jiffies + msecs_to_jiffies(wait));

		PRINT_D(GENERIC_DBG,"Current Jiffies: %lu Timeout:%llu\n",jiffies,pstrWFIDrv->p2p_mgmt_timeout);

		}

		g_linux_wlan->oup.wlan_add_mgmt_to_tx_que(mgmt_tx,mgmt_tx->buff,mgmt_tx->size,WILC_WFI_mgmt_tx_complete);
	}
      else
      {
		PRINT_D(GENERIC_DBG,"This function transmits only management frames\n");
	}
	return s32Error;
}

int   WILC_WFI_mgmt_tx_cancel_wait(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
                               struct wireless_dev *wdev,
#else
                               struct net_device *dev,
#endif
                               u64 cookie)
{
	struct WILC_WFI_priv* priv;
	struct WILC_WFIDrv * pstrWFIDrv;
	priv = wiphy_priv(wiphy);
	pstrWFIDrv = (struct WILC_WFIDrv *)priv->hWILCWFIDrv;


	PRINT_D(GENERIC_DBG,"Tx Cancel wait :%lu\n",jiffies);
	pstrWFIDrv->p2p_mgmt_timeout = jiffies;

	if(priv->bInP2PlistenState == WILC_FALSE)
	{
		/* Bug 5504: This is just to avoid connection failure when getting stuck when the supplicant
					considers the driver falsely that it is in Listen state */
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
			cfg80211_remain_on_channel_expired(priv->wdev,
				priv->strRemainOnChanParams.u64ListenCookie,
				priv->strRemainOnChanParams.pstrListenChan ,
				GFP_KERNEL);
		#else
			cfg80211_remain_on_channel_expired(priv->dev,
				priv->strRemainOnChanParams.u64ListenCookie,
				priv->strRemainOnChanParams.pstrListenChan ,
				priv->strRemainOnChanParams.tenuChannelType,
				GFP_KERNEL);
		#endif

	}

	return 0;
}

/**
*  @brief 	WILC_WFI_action
*  @details Transmit an action frame
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 JUL 2012
*  @version	1.0
* */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
int  WILC_WFI_action(struct wiphy *wiphy, struct net_device *dev,
        	struct ieee80211_channel *chan,enum nl80211_channel_type channel_type,
            const u8 *buf, size_t len, u64 *cookie)
{
	PRINT_D(HOSTAPD_DBG,"In action function\n");
	return WILC_SUCCESS;
}
#endif
#else

/**
*  @brief 	WILC_WFI_frame_register
*  @details Notify driver that a management frame type was
*     		registered. Note that this callback may not sleep, and cannot run
*      		concurrently with itself.
*  @param[in]
*  @return 	NONE.
*  @author	mdaftedar
*  @date	01 JUL 2012
*  @version
*/
void    WILC_WFI_frame_register(struct wiphy *wiphy,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
	struct wireless_dev *wdev,
#else
	struct net_device *dev,
#endif
	u16 frame_type, bool reg)
{

	struct WILC_WFI_priv* priv;
	perInterface_wlan_t* nic;


	priv = wiphy_priv(wiphy);
	nic = netdev_priv(priv->wdev->netdev);



	/*BugID_5137*/
	if(!frame_type)
		return;

	PRINT_D(GENERIC_DBG,"Frame registering Frame Type: %x: Boolean: %d\n",frame_type,reg);
	switch(frame_type)
	{
	case PROBE_REQ:
		{
			nic->g_struct_frame_reg[0].frame_type =frame_type;
			nic->g_struct_frame_reg[0].reg= reg;
		}
		break;

	case ACTION:
		{
			nic->g_struct_frame_reg[1].frame_type =frame_type;
			nic->g_struct_frame_reg[1].reg= reg;
		}
		break;

		default:
		{
				break;
		}

	}
	/*If mac is closed, then return*/
	if(!g_linux_wlan->wilc1000_initialized)
	{
		PRINT_D(GENERIC_DBG,"Return since mac is closed\n");
		return;
	}
	host_int_frame_register(priv->hWILCWFIDrv,frame_type,reg);


}
#endif
#endif /*WILC_P2P*/

/**
*  @brief 	WILC_WFI_set_cqm_rssi_config
*  @details 	Configure connection quality monitor RSSI threshold.
*  @param[in]   struct wiphy *wiphy:
*  @param[in]	struct net_device *dev:
*  @param[in]  	s32 rssi_thold:
*  @param[in]	u32 rssi_hyst:
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int    WILC_WFI_set_cqm_rssi_config(struct wiphy *wiphy,
	struct net_device *dev,  s32 rssi_thold, u32 rssi_hyst)
{
	PRINT_D(CFG80211_DBG, "Setting CQM RSSi Function\n");
	return 0;

}
/**
*  @brief 	WILC_WFI_dump_station
*  @details 	Configure connection quality monitor RSSI threshold.
*  @param[in]   struct wiphy *wiphy:
*  @param[in]	struct net_device *dev
*  @param[in]  	int idx
*  @param[in]	u8 *mac
*  @param[in]	struct station_info *sinfo
*  @return 	int : Return 0 on Success
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_dump_station(struct wiphy *wiphy, struct net_device *dev,
	int idx, u8 *mac, struct station_info *sinfo)
{
	struct WILC_WFI_priv* priv;
	PRINT_D(CFG80211_DBG, "Dumping station information\n");

	if (idx != 0)
		return -ENOENT;

	 priv = wiphy_priv(wiphy);
	//priv = netdev_priv(priv->wdev->netdev);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))	//0421
	sinfo->filled |= BIT(NL80211_STA_INFO_SIGNAL);
#else
	sinfo->filled |= STATION_INFO_SIGNAL;
#endif

	host_int_get_rssi(priv->hWILCWFIDrv, &(sinfo->signal));

#if 0
	sinfo->filled |= STATION_INFO_TX_BYTES |
			 STATION_INFO_TX_PACKETS |
			 STATION_INFO_RX_BYTES |
			 STATION_INFO_RX_PACKETS | STATION_INFO_SIGNAL | STATION_INFO_INACTIVE_TIME;

	WILC_SemaphoreAcquire(&SemHandleUpdateStats,NULL);
	sinfo->inactive_time = priv->netstats.rx_time > priv->netstats.tx_time ? jiffies_to_msecs(jiffies - priv->netstats.tx_time) : jiffies_to_msecs(jiffies - priv->netstats.rx_time);
	sinfo->rx_bytes = priv->netstats.rx_bytes;
	sinfo->tx_bytes = priv->netstats.tx_bytes;
	sinfo->rx_packets = priv->netstats.rx_packets;
	sinfo->tx_packets = priv->netstats.tx_packets;
	WILC_SemaphoreRelease(&SemHandleUpdateStats,NULL);
#endif
	return 0;

}


/**
*  @brief 	WILC_WFI_set_power_mgmt
*  @details
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 JUL 2012
*  @version	1.0WILC_WFI_set_cqmWILC_WFI_set_cqm_rssi_configWILC_WFI_set_cqm_rssi_configWILC_WFI_set_cqm_rssi_configWILC_WFI_set_cqm_rssi_config_rssi_config
*/
int WILC_WFI_set_power_mgmt(struct wiphy *wiphy, struct net_device *dev,
                                  bool enabled, int timeout)
{
	struct WILC_WFI_priv* priv;
	PRINT_D(CFG80211_DBG," Power save Enabled= %d , TimeOut = %d\n",enabled,timeout);

	if (wiphy == WILC_NULL)
		return -ENOENT;

	 priv = wiphy_priv(wiphy);
	 if(priv->hWILCWFIDrv == WILC_NULL)
 	{
 		PRINT_ER("Driver is NULL\n");
		 return -EIO;
 	}

	if(bEnablePS	== WILC_TRUE)
		host_int_set_power_mgmt(priv->hWILCWFIDrv, enabled, timeout);


	return WILC_SUCCESS;

}
#ifdef WILC_AP_EXTERNAL_MLME
/**
*  @brief 	WILC_WFI_change_virt_intf
*  @details 	Change type/configuration of virtual interface,
*      		keep the struct wireless_dev's iftype updated.
*  @param[in]   NONE
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
void wilc1000_wlan_deinit(linux_wlan_t *nic);
int wilc1000_wlan_init(struct net_device *dev, perInterface_wlan_t* p_nic);

static int WILC_WFI_change_virt_intf(struct wiphy *wiphy,struct net_device *dev,
	enum nl80211_iftype type, u32 *flags,struct vif_params *params)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	//struct WILC_WFI_mon_priv* mon_priv;
	perInterface_wlan_t* nic;
	WILC_Uint8 interface_type;
	WILC_Uint16 TID=0;
	#ifdef WILC_P2P
	WILC_Uint8 i;
	#endif

	struct net_device *net_device_1;
	struct net_device *net_device_2;
	struct WILC_WFI_priv* priv_1;
	struct WILC_WFI_priv* priv_2;
	
	net_device_1 = linux_wlan_get_if_netdev(P2P_IFC);
	net_device_2 = linux_wlan_get_if_netdev(WLAN_IFC);
	priv_1 = wdev_priv(net_device_1->ieee80211_ptr);
	priv_2 = wdev_priv(net_device_2->ieee80211_ptr);

	nic = netdev_priv(dev);
	priv = wiphy_priv(wiphy);

	PRINT_D(HOSTAPD_DBG,"In Change virtual interface function\n");
	PRINT_D(HOSTAPD_DBG,"Wireless interface name =%s\n", dev->name);
	u8P2Plocalrandom=0x01;
	u8P2Precvrandom=0x00;

	bWilc_ie= WILC_FALSE;

	#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	g_obtainingIP=WILC_FALSE;
	WILC_TimerStop(&hDuringIpTimer, WILC_NULL);
	PRINT_D(GENERIC_DBG,"Changing virtual interface, enable scan\n");
	#endif	
	switch(type)
	{
	
	case NL80211_IFTYPE_STATION:
			connecting = 0;
			PRINT_D(HOSTAPD_DBG,"Interface type = NL80211_IFTYPE_STATION\n");
		
			dev->ieee80211_ptr->iftype = type;
			priv->wdev->iftype = type;
			nic->monitor_flag = 0;
			nic->iftype = STATION_MODE;
			host_int_set_wfi_drv_handler((unsigned int)priv->hWILCWFIDrv, STATION_MODE, dev->name);
			host_int_set_operation_mode(priv->hWILCWFIDrv,STATION_MODE);
			/*Remove the enteries of the previously connected clients*/
			memset(priv->assoc_stainfo.au8Sta_AssociatedBss, 0, MAX_NUM_STA * ETH_ALEN);
	
			bEnablePS = WILC_TRUE;
			host_int_set_power_mgmt(priv_1->hWILCWFIDrv, 1, 0);
			host_int_set_power_mgmt(priv_2->hWILCWFIDrv, 1, 0);
	
			break;
			
		case NL80211_IFTYPE_P2P_CLIENT:
			connecting = 0;
			PRINT_D(HOSTAPD_DBG, "Interface type = NL80211_IFTYPE_P2P_CLIENT\n");
			

			dev->ieee80211_ptr->iftype = type;
			priv->wdev->iftype = type;
			nic->monitor_flag = 0;
			nic->iftype = CLIENT_MODE;
			bEnablePS = false;
			host_int_set_wfi_drv_handler((unsigned int)priv->hWILCWFIDrv, STATION_MODE, dev->name);
			host_int_set_operation_mode(priv->hWILCWFIDrv, STATION_MODE);
			
			host_int_set_power_mgmt(priv_1->hWILCWFIDrv, 0, 0);
			host_int_set_power_mgmt(priv_2->hWILCWFIDrv, 0, 0);
			break;
	
		case NL80211_IFTYPE_AP:
			PRINT_D(HOSTAPD_DBG,"Interface type = NL80211_IFTYPE_AP\n");
			dev->ieee80211_ptr->iftype = type;
			priv->wdev->iftype = type;
			nic->iftype = AP_MODE;
			bEnablePS = false;
		if(g_linux_wlan->wilc1000_initialized)
			{
				host_int_set_wfi_drv_handler((unsigned int)priv->hWILCWFIDrv, AP_MODE, dev->name);
				host_int_set_operation_mode(priv->hWILCWFIDrv, AP_MODE);
				host_int_set_power_mgmt(priv_1->hWILCWFIDrv, 0, 0);
				host_int_set_power_mgmt(priv_2->hWILCWFIDrv, 0, 0);
			}
			break;
					
		case NL80211_IFTYPE_P2P_GO:
			PRINT_D(HOSTAPD_DBG,"Interface type = NL80211_IFTYPE_GO\n");
			PRINT_D(GENERIC_DBG,"start duringIP timer\n");
	
		#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
			g_obtainingIP=WILC_TRUE;
			WILC_TimerStart(&hDuringIpTimer, duringIP_TIME, WILC_NULL, WILC_NULL);
		#endif	

			dev->ieee80211_ptr->iftype = type;
			priv->wdev->iftype = type;
			nic->iftype = GO_MODE;
			host_int_set_wfi_drv_handler((unsigned int)priv->hWILCWFIDrv, AP_MODE, dev->name);
			host_int_set_operation_mode(priv->hWILCWFIDrv, AP_MODE);
			bEnablePS = WILC_FALSE;
			host_int_set_power_mgmt(priv_1->hWILCWFIDrv, 0, 0);
			host_int_set_power_mgmt(priv_2->hWILCWFIDrv, 0, 0);
			break;
			
		default:
			PRINT_ER("Unknown interface type= %d\n", type);
			s32Error = -EINVAL;
			return s32Error;
			break;
		}
	
		return s32Error;
	}
	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
/* (austin.2013-07-23)

	To support revised cfg80211_ops

		add_beacon --> start_ap
		set_beacon --> change_beacon
		del_beacon --> stop_ap

		beacon_parameters  -->	cfg80211_ap_settings
								cfg80211_beacon_data

	applicable for linux kernel 3.4+
*/

/**
*  @brief 	WILC_WFI_start_ap
*  @details 	Add a beacon with given parameters, @head, @interval
*      		and @dtim_period will be valid, @tail is optional.
*  @param[in]   wiphy
*  @param[in]   dev	The net device structure
*  @param[in]   settings	cfg80211_ap_settings parameters for the beacon to be added
*  @return 	int : Return 0 on Success.
*  @author	austin
*  @date	23 JUL 2013
*  @version	1.0
*/
static int WILC_WFI_start_ap(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_ap_settings *settings)
{
	struct cfg80211_beacon_data* beacon = &(settings->beacon);
	struct WILC_WFI_priv* priv;
	WILC_Sint32 s32Error = WILC_SUCCESS;
	perInterface_wlan_t* nic;	
		
	priv = wiphy_priv(wiphy);
	nic = netdev_priv(dev);
	PRINT_D(HOSTAPD_DBG,"Starting ap\n");

	PRINT_D(HOSTAPD_DBG,"Interval = %d \n DTIM period = %d\n Head length = %d Tail length = %d\n",
		settings->beacon_interval , settings->dtim_period, beacon->head_len, beacon->tail_len );

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	s32Error = WILC_WFI_CfgSetChannel(wiphy, &settings->chandef);

	if(s32Error != WILC_SUCCESS)
		PRINT_ER("Error in setting channel\n");
#endif
	linux_wlan_set_bssid(dev,g_linux_wlan->strInterfaceInfo[nic->u8IfIdx].aSrcAddress,AP_MODE);
	/*disable PS  in case of AP*/
	host_int_set_power_mgmt(priv->hWILCWFIDrv, 0, 0);
	#ifndef WILC_FULLY_HOSTING_AP
	s32Error = host_int_add_beacon(	priv->hWILCWFIDrv,
									settings->beacon_interval,
									settings->dtim_period,
									beacon->head_len, (WILC_Uint8*)beacon->head,
									beacon->tail_len, (WILC_Uint8*)beacon->tail);
	#else
	s32Error = host_add_beacon(	priv->hWILCWFIDrv,
									settings->beacon_interval,
									settings->dtim_period,
									beacon->head_len, (WILC_Uint8*)beacon->head,
									beacon->tail_len, (WILC_Uint8*)beacon->tail);
	#endif

	return s32Error;
}

/**
*  @brief 	WILC_WFI_change_beacon
*  @details 	Add a beacon with given parameters, @head, @interval
*      		and @dtim_period will be valid, @tail is optional.
*  @param[in]   wiphy
*  @param[in]   dev	The net device structure
*  @param[in]   beacon	cfg80211_beacon_data for the beacon to be changed
*  @return 	int : Return 0 on Success.
*  @author	austin
*  @date	23 JUL 2013
*  @version	1.0
*/
static int  WILC_WFI_change_beacon(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_beacon_data *beacon)
{
	struct WILC_WFI_priv* priv;
	WILC_Sint32 s32Error = WILC_SUCCESS;

	priv = wiphy_priv(wiphy);
	PRINT_D(HOSTAPD_DBG,"Setting beacon\n");


#ifndef WILC_FULLY_HOSTING_AP
	s32Error = host_int_add_beacon(	priv->hWILCWFIDrv,
									0,
									0,
									beacon->head_len, (WILC_Uint8*)beacon->head,
									beacon->tail_len, (WILC_Uint8*)beacon->tail);
#else
	s32Error = host_add_beacon(	priv->hWILCWFIDrv,
									0,
									0,
									beacon->head_len, (WILC_Uint8*)beacon->head,
									beacon->tail_len, (WILC_Uint8*)beacon->tail);
#endif

	return s32Error;
}

/**
*  @brief 	WILC_WFI_stop_ap
*  @details 	Remove beacon configuration and stop sending the beacon.
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	austin
*  @date	23 JUL 2013
*  @version	1.0
*/
static int  WILC_WFI_stop_ap(struct wiphy *wiphy, struct net_device *dev)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	WILC_Uint8 NullBssid[ETH_ALEN] = {0};


	WILC_NULLCHECK(s32Error, wiphy);

	priv = wiphy_priv(wiphy);

	PRINT_D(HOSTAPD_DBG,"Deleting beacon\n");

	/*BugID_5188*/
	linux_wlan_set_bssid(dev, NullBssid,AP_MODE);

	#ifndef WILC_FULLY_HOSTING_AP
	s32Error = host_int_del_beacon(priv->hWILCWFIDrv);
	#else
	s32Error = host_del_beacon(priv->hWILCWFIDrv);
	#endif

	WILC_ERRORCHECK(s32Error);

	WILC_CATCH(s32Error)
	{
	}
	return s32Error;
}

#else /* here belows are original for < kernel 3.3 (austin.2013-07-23) */

/**
*  @brief 	WILC_WFI_add_beacon
*  @details 	Add a beacon with given parameters, @head, @interval
*      		and @dtim_period will be valid, @tail is optional.
*  @param[in]   wiphy
*  @param[in]   dev	The net device structure
*  @param[in]   info	Parameters for the beacon to be added
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_add_beacon(struct wiphy *wiphy, struct net_device *dev,
	struct beacon_parameters *info)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	perInterface_wlan_t* nic;



	priv = wiphy_priv(wiphy);
	nic = netdev_priv(dev);
	PRINT_D(HOSTAPD_DBG,"Adding Beacon\n");

	PRINT_D(HOSTAPD_DBG,"Interval = %d \n DTIM period = %d\n Head length = %d Tail length = %d\n",info->interval , info->dtim_period,info->head_len,info->tail_len );

	linux_wlan_set_bssid(dev,g_linux_wlan->strInterfaceInfo[nic->u8IfIdx].aSrcAddress,AP_MODE);
	host_int_set_power_mgmt(priv->hWILCWFIDrv, 0, 0);
	#ifndef WILC_FULLY_HOSTING_AP
	s32Error = host_int_add_beacon(priv->hWILCWFIDrv, info->interval,
									 info->dtim_period,
									 info->head_len, info->head,
									 info->tail_len, info->tail);

	#else
	s32Error = host_add_beacon(priv->hWILCWFIDrv, info->interval,
									 info->dtim_period,
									 info->head_len, info->head,
									 info->tail_len, info->tail);
	#endif

	return s32Error;
}

/**
*  @brief 	WILC_WFI_set_beacon
*  @details 	Change the beacon parameters for an access point mode
*      		interface. This should reject the call when no beacon has been
*      		configured.
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int  WILC_WFI_set_beacon(struct wiphy *wiphy, struct net_device *dev,
	struct beacon_parameters *info)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;


	PRINT_D(HOSTAPD_DBG,"Setting beacon\n");

	s32Error = WILC_WFI_add_beacon(wiphy, dev, info);

	return s32Error;
}

/**
*  @brief 	WILC_WFI_del_beacon
*  @details 	Remove beacon configuration and stop sending the beacon.
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int  WILC_WFI_del_beacon(struct wiphy *wiphy, struct net_device *dev)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	WILC_Uint8 NullBssid[ETH_ALEN] = {0};


	WILC_NULLCHECK(s32Error, wiphy);

	priv = wiphy_priv(wiphy);

	PRINT_D(HOSTAPD_DBG,"Deleting beacon\n");

	/*BugID_5188*/
	linux_wlan_set_bssid(dev, NullBssid,AP_MODE);

	#ifndef WILC_FULLY_HOSTING_AP
	s32Error = host_int_del_beacon(priv->hWILCWFIDrv);
	#else
	s32Error = host_del_beacon(priv->hWILCWFIDrv);
	#endif

	WILC_ERRORCHECK(s32Error);

	WILC_CATCH(s32Error)
	{
	}
	return s32Error;
}

#endif	/* linux kernel 3.4+ (austin.2013-07-23) */

/**
*  @brief 	WILC_WFI_add_station
*  @details 	Add a new station.
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int  WILC_WFI_add_station(struct wiphy *wiphy, struct net_device *dev,
	u8 *mac, struct station_parameters *params)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	struct WILC_AddStaParam strStaParams={{0}};
	perInterface_wlan_t* nic;


	WILC_NULLCHECK(s32Error, wiphy);

	priv = wiphy_priv(wiphy);
	nic = netdev_priv(dev);

	if(nic->iftype == AP_MODE ||nic->iftype == GO_MODE )
	{
		#ifndef WILC_FULLY_HOSTING_AP

		WILC_memcpy(strStaParams.au8BSSID, mac, ETH_ALEN);
		WILC_memcpy(priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid],mac,ETH_ALEN);
		strStaParams.u16AssocID = params->aid;
		strStaParams.u8NumRates = params->supported_rates_len;
		strStaParams.pu8Rates = params->supported_rates;

		PRINT_D(CFG80211_DBG,"Adding station parameters %d\n",params->aid);

		PRINT_D(CFG80211_DBG,"BSSID = %x%x%x%x%x%x\n",priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][0],priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][1],priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][2],priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][3],priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][4],
			priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][5]);
		PRINT_D(HOSTAPD_DBG,"ASSOC ID = %d\n",strStaParams.u16AssocID);
		PRINT_D(HOSTAPD_DBG,"Number of supported rates = %d\n",strStaParams.u8NumRates);

		if(params->ht_capa == WILC_NULL)
		{
			strStaParams.bIsHTSupported = WILC_FALSE;
		}
		else
		{
			strStaParams.bIsHTSupported = WILC_TRUE;
			strStaParams.u16HTCapInfo = params->ht_capa->cap_info;
			strStaParams.u8AmpduParams = params->ht_capa->ampdu_params_info;
			WILC_memcpy(strStaParams.au8SuppMCsSet, &params->ht_capa->mcs, WILC_SUPP_MCS_SET_SIZE);
			strStaParams.u16HTExtParams = params->ht_capa->extended_ht_cap_info;
			strStaParams.u32TxBeamformingCap = params->ht_capa->tx_BF_cap_info;
			strStaParams.u8ASELCap = params->ht_capa->antenna_selection_info;
		}

		strStaParams.u16FlagsMask = params->sta_flags_mask;
		strStaParams.u16FlagsSet = params->sta_flags_set;

		PRINT_D(HOSTAPD_DBG,"IS HT supported = %d\n", strStaParams.bIsHTSupported);
		PRINT_D(HOSTAPD_DBG,"Capability Info = %d\n", strStaParams.u16HTCapInfo);
		PRINT_D(HOSTAPD_DBG,"AMPDU Params = %d\n",strStaParams.u8AmpduParams);
		PRINT_D(HOSTAPD_DBG,"HT Extended params = %d\n",strStaParams.u16HTExtParams);
		PRINT_D(HOSTAPD_DBG,"Tx Beamforming Cap = %d\n",strStaParams.u32TxBeamformingCap);
		PRINT_D(HOSTAPD_DBG,"Antenna selection info = %d\n",strStaParams.u8ASELCap);
		PRINT_D(HOSTAPD_DBG,"Flag Mask = %d\n",strStaParams.u16FlagsMask);
		PRINT_D(HOSTAPD_DBG,"Flag Set = %d\n",strStaParams.u16FlagsSet);

		s32Error = host_int_add_station(priv->hWILCWFIDrv, &strStaParams);
		WILC_ERRORCHECK(s32Error);

		#else
		PRINT_D(CFG80211_DBG,"Adding station parameters %d\n",params->aid);
		WILC_memcpy(priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid],mac,ETH_ALEN);

		PRINT_D(CFG80211_DBG,"BSSID = %x%x%x%x%x%x\n",priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][0],priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][1],priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][2],priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][3],priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][4],
			priv->assoc_stainfo.au8Sta_AssociatedBss[params->aid][5]);

		WILC_AP_AddSta(mac, params);
		WILC_ERRORCHECK(s32Error);
		#endif //WILC_FULLY_HOSTING_AP

	}

	WILC_CATCH(s32Error)
	{
	}
	return s32Error;
}

/**
*  @brief 	WILC_WFI_del_station
*  @details 	Remove a station; @mac may be NULL to remove all stations.
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_del_station(struct wiphy *wiphy, struct net_device *dev,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
				struct station_del_parameters *params)
#else
				u8 *mac)
#endif
{
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	u8 *mac = params->mac;
	#endif
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	perInterface_wlan_t* nic;
	WILC_NULLCHECK(s32Error, wiphy);
	/*BugID_4795: mac may be null pointer to indicate deleting all stations, so avoid null check*/
	//WILC_NULLCHECK(s32Error, mac);

	priv = wiphy_priv(wiphy);
	nic = netdev_priv(dev);

	if(nic->iftype == AP_MODE || nic->iftype == GO_MODE)
	{
		PRINT_D(HOSTAPD_DBG,"Deleting station\n");


		if(mac == WILC_NULL)
		{
			PRINT_D(HOSTAPD_DBG,"All associated stations \n");
			s32Error = host_int_del_allstation(priv->hWILCWFIDrv , priv->assoc_stainfo.au8Sta_AssociatedBss);
		}
		else
		{
			PRINT_D(HOSTAPD_DBG,"With mac address: %x%x%x%x%x%x\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
		}

		#ifndef WILC_FULLY_HOSTING_AP
		s32Error = host_int_del_station(priv->hWILCWFIDrv , mac);
		#else
		WILC_AP_RemoveSta(mac);
		#endif //WILC_FULLY_HOSTING_AP

		WILC_ERRORCHECK(s32Error);
	}
	WILC_CATCH(s32Error)
	{
	}
	return s32Error;
}

/**
*  @brief 	WILC_WFI_change_station
*  @details 	Modify a given station.
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
static int WILC_WFI_change_station(struct wiphy *wiphy, struct net_device *dev,
		u8 *mac, struct station_parameters *params)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv;
	struct WILC_AddStaParam strStaParams={{0}};
	perInterface_wlan_t* nic;


	PRINT_D(HOSTAPD_DBG,"Change station paramters\n");

	WILC_NULLCHECK(s32Error, wiphy);

	priv = wiphy_priv(wiphy);
	nic = netdev_priv(dev);

	if(nic->iftype == AP_MODE || nic->iftype == GO_MODE)
	{
		#ifndef WILC_FULLY_HOSTING_AP

		WILC_memcpy(strStaParams.au8BSSID, mac, ETH_ALEN);
		strStaParams.u16AssocID = params->aid;
		strStaParams.u8NumRates = params->supported_rates_len;
		strStaParams.pu8Rates = params->supported_rates;

		PRINT_D(HOSTAPD_DBG,"BSSID = %x%x%x%x%x%x\n",strStaParams.au8BSSID[0],strStaParams.au8BSSID[1],strStaParams.au8BSSID[2],strStaParams.au8BSSID[3],strStaParams.au8BSSID[4],
			strStaParams.au8BSSID[5]);
		PRINT_D(HOSTAPD_DBG,"ASSOC ID = %d\n",strStaParams.u16AssocID);
		PRINT_D(HOSTAPD_DBG,"Number of supported rates = %d\n",strStaParams.u8NumRates);

		if(params->ht_capa == WILC_NULL)
		{
			strStaParams.bIsHTSupported = WILC_FALSE;
		}
		else
		{
			strStaParams.bIsHTSupported = WILC_TRUE;
			strStaParams.u16HTCapInfo = params->ht_capa->cap_info;
			strStaParams.u8AmpduParams = params->ht_capa->ampdu_params_info;
			WILC_memcpy(strStaParams.au8SuppMCsSet, &params->ht_capa->mcs, WILC_SUPP_MCS_SET_SIZE);
			strStaParams.u16HTExtParams = params->ht_capa->extended_ht_cap_info;
			strStaParams.u32TxBeamformingCap = params->ht_capa->tx_BF_cap_info;
			strStaParams.u8ASELCap = params->ht_capa->antenna_selection_info;

		}

		strStaParams.u16FlagsMask = params->sta_flags_mask;
		strStaParams.u16FlagsSet = params->sta_flags_set;

		PRINT_D(HOSTAPD_DBG,"IS HT supported = %d\n", strStaParams.bIsHTSupported);
		PRINT_D(HOSTAPD_DBG,"Capability Info = %d\n", strStaParams.u16HTCapInfo);
		PRINT_D(HOSTAPD_DBG,"AMPDU Params = %d\n",strStaParams.u8AmpduParams);
		PRINT_D(HOSTAPD_DBG,"HT Extended params = %d\n",strStaParams.u16HTExtParams);
		PRINT_D(HOSTAPD_DBG,"Tx Beamforming Cap = %d\n",strStaParams.u32TxBeamformingCap);
		PRINT_D(HOSTAPD_DBG,"Antenna selection info = %d\n",strStaParams.u8ASELCap);
		PRINT_D(HOSTAPD_DBG,"Flag Mask = %d\n",strStaParams.u16FlagsMask);
		PRINT_D(HOSTAPD_DBG,"Flag Set = %d\n",strStaParams.u16FlagsSet);

		s32Error = host_int_edit_station(priv->hWILCWFIDrv, &strStaParams);
		WILC_ERRORCHECK(s32Error);

		#else
		WILC_AP_EditSta(mac, params);
		WILC_ERRORCHECK(s32Error);
		#endif //WILC_FULLY_HOSTING_AP

	}
	WILC_CATCH(s32Error)
	{
	}
	return s32Error;
}


/**
*  @brief 	WILC_WFI_add_virt_intf
*  @details
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 JUL 2012
*  @version	1.0
*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
struct wireless_dev *WILC_WFI_add_virt_intf(struct wiphy *wiphy, const char *name,
						unsigned char name_assign_type,
						enum nl80211_iftype type, u32 *flags,
						struct vif_params *params)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)         /* tony for v3.8 support */
struct wireless_dev *WILC_WFI_add_virt_intf(struct wiphy *wiphy, const char *name,
					    enum nl80211_iftype type, u32 *flags,
					    struct vif_params *params)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)       /* tony for v3.6 support */
struct wireless_dev *WILC_WFI_add_virt_intf(struct wiphy *wiphy, char *name,
					    enum nl80211_iftype type, u32 *flags,
					    struct vif_params *params)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
int WILC_WFI_add_virt_intf(struct wiphy *wiphy, char *name,
               		enum nl80211_iftype type, u32 *flags,
                      	struct vif_params *params)
#else
struct net_device *WILC_WFI_add_virt_intf(struct wiphy *wiphy, char *name,
  	                        enum nl80211_iftype type, u32 *flags,
        	                struct vif_params *params)
#endif
{
	perInterface_wlan_t * nic;
	struct WILC_WFI_priv* priv;
	//struct WILC_WFI_mon_priv* mon_priv;
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
	WILC_Sint32 s32Error = WILC_SUCCESS;
	#endif
	struct net_device * new_ifc = NULL;
	priv = wiphy_priv(wiphy);



	PRINT_D(HOSTAPD_DBG,"Adding monitor interface[%p]\n",priv->wdev->netdev);

	nic = netdev_priv(priv->wdev->netdev);


	if(type == NL80211_IFTYPE_MONITOR)
	{
		PRINT_D(HOSTAPD_DBG,"Monitor interface mode: Initializing mon interface virtual device driver\n");
		PRINT_D(HOSTAPD_DBG,"Adding monitor interface[%p]\n",nic->wilc_netdev);
		new_ifc = WILC_WFI_init_mon_interface(name,nic->wilc_netdev);
		if(new_ifc != NULL)
		{
			PRINT_D(HOSTAPD_DBG,"Setting monitor flag in private structure\n");
			#ifdef SIMULATION
			priv = netdev_priv(priv->wdev->netdev);
			priv->monitor_flag = 1;
			#else
			nic = netdev_priv(priv->wdev->netdev);
			nic->monitor_flag = 1;
			#endif
		}
		else
			PRINT_ER("Error in initializing monitor interface\n ");
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)	/* tony for v3.8 support */
	return priv->wdev;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
	return s32Error;
#else
	//return priv->wdev->netdev;
	PRINT_D(HOSTAPD_DBG,"IFC[%p] created\n",new_ifc);
	return new_ifc;
#endif

}

/**
*  @brief 	WILC_WFI_del_virt_intf
*  @details
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 JUL 2012
*  @version	1.0
*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
int WILC_WFI_del_virt_intf(struct wiphy *wiphy, struct wireless_dev *wdev)	/* tony for v3.8 support */
#else
int WILC_WFI_del_virt_intf(struct wiphy *wiphy,struct net_device *dev)
#endif
{
	PRINT_D(HOSTAPD_DBG,"Deleting virtual interface\n");
		return WILC_SUCCESS;
}

uint8_t u8SuspendOnEvent = 0;
int WILC_WFI_suspend(struct wiphy *wiphy, struct cfg80211_wowlan *wow)
{
	printk("CFG suspend [%x]\n",wow);
	/*TODO:Cancel any ongoing connection or scan*/
	if(wow != NULL && linux_wlan_get_num_conn_ifcs())
		u8SuspendOnEvent = 1;
	else
		u8SuspendOnEvent = 0;
	return 0;
}
int WILC_WFI_resume(struct wiphy *wiphy)
{
	printk("CFG TESUME\n");
	return 0;
}

void	WILC_WFI_wake_up(struct wiphy *wiphy, bool enabled)
{
	printk("Set wake up = %d\n",enabled);
}

int WILC_WFI_set_tx_power(struct wiphy *wiphy,
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	struct wireless_dev *wdev,
	#endif
    enum nl80211_tx_power_setting type, int mbm)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	WILC_Sint32 tx_power = MBM_TO_DBM(mbm);
	struct WILC_WFI_priv* priv = wiphy_priv(wiphy);

	PRINT_D(CFG80211_DBG, "Setting tx power to %d\n", tx_power);
	if(tx_power < 0)
		tx_power = 0;
	else if(tx_power > 18)
		tx_power = 18;
	s32Error = host_int_set_tx_power(priv->hWILCWFIDrv,(WILC_Uint8)tx_power);

	return s32Error;
}
int WILC_WFI_get_tx_power(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	struct wireless_dev *wdev,
#endif
	int *dbm)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv = wiphy_priv(wiphy);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	if(!g_linux_wlan->wilc1000_initialized)
		return WILC_FAIL;
#endif

	*dbm=0;
	s32Error = host_int_get_tx_power(priv->hWILCWFIDrv, (WILC_Uint8*)(dbm));
	PRINT_D(CFG80211_DBG, "Got tx power %d\n", *dbm);

	return s32Error;
}

int WILC_WFI_set_antenna(struct wiphy *wiphy, u32 tx_ant, u32 rx_ant)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv* priv = wiphy_priv(wiphy);
	
	PRINT_D(CFG80211_DBG,"Select antenna mode %d\n",tx_ant);
	s32Error = host_int_set_antenna(priv->hWILCWFIDrv,(WILC_Uint8)tx_ant);

	return s32Error;
}

#endif /*ATWILC_AP_EXTERNAL_MLME*/
static struct cfg80211_ops WILC_WFI_cfg80211_ops = {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
	/*
	*	replaced set_channel by set_monitor_channel
	*	from v3.6
	*	tony, 2013-10-29
	*/
	.set_monitor_channel = WILC_WFI_CfgSetChannel,
#else
	.set_channel = WILC_WFI_CfgSetChannel,
#endif
	.scan = WILC_WFI_CfgScan,
	.connect = WILC_WFI_CfgConnect,
	.disconnect = WILC_WFI_disconnect,
	.add_key = WILC_WFI_add_key,
	.del_key = WILC_WFI_del_key,
	.get_key = WILC_WFI_get_key,
	.set_default_key = WILC_WFI_set_default_key,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
	//.dump_survey = WILC_WFI_dump_survey,
#endif
	#ifdef WILC_AP_EXTERNAL_MLME
	.add_virtual_intf = WILC_WFI_add_virt_intf,
	.del_virtual_intf = WILC_WFI_del_virt_intf,
	.change_virtual_intf = WILC_WFI_change_virt_intf,

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
	.add_beacon = WILC_WFI_add_beacon,
	.set_beacon = WILC_WFI_set_beacon,
	.del_beacon = WILC_WFI_del_beacon,
#else
	/* supports kernel 3.4+ change. austin.2013-07-23 */
	.start_ap = WILC_WFI_start_ap,
	.change_beacon = WILC_WFI_change_beacon,
	.stop_ap = WILC_WFI_stop_ap,
#endif
	.add_station = WILC_WFI_add_station,
	.del_station = WILC_WFI_del_station,
	.change_station = WILC_WFI_change_station,
	#endif /* WILC_AP_EXTERNAL_MLME*/
	#ifndef WILC_FULLY_HOSTING_AP
	.get_station = WILC_WFI_get_station,
	#endif
	.dump_station = WILC_WFI_dump_station,
	.change_bss = WILC_WFI_change_bss,
	//.auth = WILC_WFI_auth,
	//.assoc = WILC_WFI_assoc,
	//.deauth = WILC_WFI_deauth,
	//.disassoc = WILC_WFI_disassoc,
	.set_wiphy_params = WILC_WFI_set_wiphy_params,

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
	//.set_bitrate_mask = WILC_WFI_set_bitrate_mask,
	.set_pmksa = WILC_WFI_set_pmksa,
	.del_pmksa = WILC_WFI_del_pmksa,
	.flush_pmksa = WILC_WFI_flush_pmksa,
#ifdef WILC_P2P
	.remain_on_channel = WILC_WFI_remain_on_channel,
	.cancel_remain_on_channel = WILC_WFI_cancel_remain_on_channel,
	.mgmt_tx_cancel_wait = WILC_WFI_mgmt_tx_cancel_wait,
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
	.action = WILC_WFI_action,
	#endif
	#else
	.mgmt_tx = WILC_WFI_mgmt_tx,
	.mgmt_frame_register = WILC_WFI_frame_register,
	#endif
#endif
	//.mgmt_tx_cancel_wait = WILC_WFI_mgmt_tx_cancel_wait,
#if (defined WILC_SPI)
#else
	.set_power_mgmt = WILC_WFI_set_power_mgmt,
#endif
	.set_cqm_rssi_config = WILC_WFI_set_cqm_rssi_config,
#endif
	.suspend = WILC_WFI_suspend,
	.resume = WILC_WFI_resume,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
	.set_wakeup = WILC_WFI_wake_up,
#endif
	.set_tx_power = WILC_WFI_set_tx_power,
	.get_tx_power = WILC_WFI_get_tx_power,
	.set_antenna= WILC_WFI_set_antenna,
};





/**
*  @brief 	WILC_WFI_update_stats
*  @details 	Modify parameters for a given BSS.
*  @param[in]
*  @return 	int : Return 0 on Success.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0WILC_WFI_set_cqmWILC_WFI_set_cqm_rssi_configWILC_WFI_set_cqm_rssi_configWILC_WFI_set_cqm_rssi_configWILC_WFI_set_cqm_rssi_config_rssi_config
*/
int WILC_WFI_update_stats(struct wiphy *wiphy, u32 pktlen , u8 changed)
{

	struct WILC_WFI_priv *priv;

	priv = wiphy_priv(wiphy);
	//WILC_SemaphoreAcquire(&SemHandleUpdateStats,NULL);
#if 1
	switch(changed)
	{

		case WILC_WFI_RX_PKT:
		{
			//MI_PRINTF("In Rx Receive Packet\n");
			priv->netstats.rx_packets++;
			priv->netstats.rx_bytes += pktlen;
			priv->netstats.rx_time = get_jiffies_64();
		}
		break;
		case WILC_WFI_TX_PKT:
		{
			priv->netstats.tx_packets++;
			priv->netstats.tx_bytes += pktlen;
			priv->netstats.tx_time = get_jiffies_64();

		}
		break;

		default:
			break;
	}
	//WILC_SemaphoreRelease(&SemHandleUpdateStats,NULL);
#endif
	return 0;
}
/**
*  @brief 	WILC_WFI_InitPriv
*  @details 	Initialization of the net device, private data
*  @param[in]   NONE
*  @return 	NONE
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
void WILC_WFI_InitPriv(struct net_device *dev)
{

	struct WILC_WFI_priv *priv;
	priv = netdev_priv(dev);

	priv->netstats.rx_packets = 0;
	priv->netstats.tx_packets = 0;
	priv->netstats.rx_bytes = 0;
	priv->netstats.rx_bytes = 0;
	priv->netstats.rx_time = 0;
	priv->netstats.tx_time = 0;


}
/**
*  @brief 	WILC_WFI_CfgAlloc
*  @details 	Allocation of the wireless device structure and assigning it
*		to the cfg80211 operations structure.
*  @param[in]   NONE
*  @return 	wireless_dev : Returns pointer to wireless_dev structure.
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
struct wireless_dev* WILC_WFI_CfgAlloc(void)
{

	struct wireless_dev *wdev;


	PRINT_D(CFG80211_DBG,"Allocating wireless device\n");
 	/*Allocating the wireless device structure*/
 	wdev = kzalloc(sizeof(struct wireless_dev), GFP_KERNEL);
	if (!wdev)
	{
		PRINT_ER("Cannot allocate wireless device\n");
		goto _fail_;
	}

	/*Creating a new wiphy, linking wireless structure with the wiphy structure*/
	wdev->wiphy = wiphy_new(&WILC_WFI_cfg80211_ops, sizeof(struct WILC_WFI_priv));
	if (!wdev->wiphy)
	{
		PRINT_ER("Cannot allocate wiphy\n");
		goto _fail_mem_;

	}

	#ifdef WILC_AP_EXTERNAL_MLME
	//enable 802.11n HT
	WILC_WFI_band_2ghz.ht_cap.ht_supported = 1;
	WILC_WFI_band_2ghz.ht_cap.cap |= (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT);
	WILC_WFI_band_2ghz.ht_cap.mcs.rx_mask[0] = 0xff;
	WILC_WFI_band_2ghz.ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_8K;
	WILC_WFI_band_2ghz.ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;
	#endif

	/*wiphy bands*/
	wdev->wiphy->bands[IEEE80211_BAND_2GHZ]= &WILC_WFI_band_2ghz;

	return wdev;

_fail_mem_:
	kfree(wdev);
_fail_:
	return NULL;

}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
static const struct wiphy_wowlan_support wowlan_support = {
        .flags = WIPHY_WOWLAN_ANY
};
#endif
/**
*  @brief 	WILC_WFI_WiphyRegister
*  @details 	Registering of the wiphy structure and interface modes
*  @param[in]   NONE
*  @return 	NONE
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
struct wireless_dev* WILC_WFI_WiphyRegister(struct net_device *net)
{
	struct WILC_WFI_priv *priv;
	struct wireless_dev *wdev;
	WILC_Sint32 s32Error = WILC_SUCCESS;

	PRINT_D(CFG80211_DBG,"Registering wifi device\n");

	wdev = WILC_WFI_CfgAlloc();
	if(wdev == NULL){
		PRINT_ER("CfgAlloc Failed\n");
		return NULL;
		}


	/*Return hardware description structure (wiphy)'s priv*/
	priv = wdev_priv(wdev);
	WILC_SemaphoreCreate(&(priv->SemHandleUpdateStats),NULL);

	/*Link the wiphy with wireless structure*/
	priv->wdev = wdev;

	/*Maximum number of probed ssid to be added by user for the scan request*/
	wdev->wiphy->max_scan_ssids = MAX_NUM_PROBED_SSID;
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
	wdev->wiphy->wowlan = &wowlan_support;
	#elif LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0)
	wdev->wiphy->wowlan.flags = WIPHY_WOWLAN_ANY;
	#endif
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
	/*Maximum number of pmkids to be cashed*/
	wdev->wiphy->max_num_pmkids = WILC_MAX_NUM_PMKIDS;
	PRINT_INFO(CFG80211_DBG,"Max number of PMKIDs = %d\n",wdev->wiphy->max_num_pmkids);
	#endif

	wdev->wiphy->max_scan_ie_len = 1000;

	/*signal strength in mBm (100*dBm) */
	wdev->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

	/*Set the availaible cipher suites*/
	wdev->wiphy->cipher_suites = cipher_suites;
	wdev->wiphy->n_cipher_suites = ARRAY_SIZE(cipher_suites);

	/*bitmap of antennas which are available to be configured as TX or RX antennas
	   (3) means both antennas are available for TX and RX */
	wdev->wiphy->available_antennas_tx=0x3;
	wdev->wiphy->available_antennas_rx=0x3;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
	/*Setting default managment types: for register action frame:  */
	wdev->wiphy->mgmt_stypes = wilc_wfi_cfg80211_mgmt_types;
#endif

#ifdef WILC_P2P
	wdev->wiphy->max_remain_on_channel_duration = 500;
	/*Setting the wiphy interfcae mode and type before registering the wiphy*/
	wdev->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) | BIT(NL80211_IFTYPE_AP) | BIT(NL80211_IFTYPE_MONITOR) | BIT(NL80211_IFTYPE_P2P_GO) |
		BIT(NL80211_IFTYPE_P2P_CLIENT);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,3,0)

	wdev->wiphy->flags |= WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL;
#endif
#else
	wdev->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) | BIT(NL80211_IFTYPE_AP) | BIT(NL80211_IFTYPE_MONITOR);
#endif
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,39)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
	wdev->wiphy->flags |= WIPHY_FLAG_SUPPORTS_SEPARATE_DEFAULT_KEYS;
#endif
#endif
	wdev->iftype = NL80211_IFTYPE_STATION;



	PRINT_INFO(CFG80211_DBG,"Max scan ids = %d,Max scan IE len = %d,Signal Type = %d,Interface Modes = %d,Interface Type = %d\n",
			wdev->wiphy->max_scan_ssids,wdev->wiphy->max_scan_ie_len,wdev->wiphy->signal_type,
			wdev->wiphy->interface_modes, wdev->iftype);

	#ifdef WILC_SDIO
    set_wiphy_dev(wdev->wiphy, &local_sdio_func->dev); //tony
	#endif

	/*Register wiphy structure*/
	s32Error = wiphy_register(wdev->wiphy);
	if (s32Error){
		PRINT_ER("Cannot register wiphy device\n");
		/*should define what action to be taken in such failure*/
		}
	else
	{
		PRINT_D(CFG80211_DBG,"Successful Registering\n");
	}

#if 0
	/*wdev[i]->wiphy->interface_modes =
			BIT(NL80211_IFTYPE_AP);
		wdev[i]->iftype = NL80211_IFTYPE_AP;
	*/

	/*Pointing the priv structure the netdev*/
    priv= netdev_priv(net);

	/*linking the wireless_dev structure with the netdevice*/
	priv->dev->ieee80211_ptr = wdev;
	priv->dev->ml_priv = priv;
	wdev->netdev = priv->dev;
#endif
	priv->dev = net;
#if 0
	ret = host_int_init(&priv->hWILCWFIDrv);
	if(ret)
	{
		PRINT_ER("Error Init Driver\n");
	}
#endif
	return wdev;


}
/**
*  @brief 	WILC_WFI_WiphyFree
*  @details 	Freeing allocation of the wireless device structure
*  @param[in]   NONE
*  @return 	NONE
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
int WILC_WFI_InitHostInt(struct net_device *net)
{

	WILC_Sint32 s32Error = WILC_SUCCESS;

	struct WILC_WFI_priv *priv;

	tstrWILC_SemaphoreAttrs strSemaphoreAttrs;

	PRINT_D(INIT_DBG,"Host[%p][%p]\n",net,net->ieee80211_ptr);
	priv = wdev_priv(net->ieee80211_ptr);
	if(op_ifcs==0)
	{
		s32Error = WILC_TimerCreate(&(hAgingTimer), remove_network_from_shadow, WILC_NULL);
		#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
		s32Error = WILC_TimerCreate(&(hDuringIpTimer), clear_duringIP, WILC_NULL);
		#endif

		/*TicketId1001*/
		/*Timer to handle eapol 1/4 buffering if needed*/
		setup_timer(&(hEAPFrameBuffTimer), EAP_buff_timeout, 0);
	}
	op_ifcs++;
	if(s32Error < 0){
		PRINT_ER("Failed to creat refresh Timer\n");
		return s32Error;
	}

	WILC_SemaphoreFillDefault(&strSemaphoreAttrs);

	/////////////////////////////////////////
	//strSemaphoreAttrs.u32InitCount = 0;


	priv->gbAutoRateAdjusted = WILC_FALSE;

	priv->bInP2PlistenState = WILC_FALSE;

	WILC_SemaphoreCreate(&(priv->hSemScanReq), &strSemaphoreAttrs);
	s32Error = host_int_init(&priv->hWILCWFIDrv);
	//s32Error = host_int_init(&priv->hWILCWFIDrv_2);
	if(s32Error)
	{
		PRINT_ER("Error while initializing hostinterface\n");
	}
	return s32Error;
}

/**
*  @brief 	WILC_WFI_WiphyFree
*  @details 	Freeing allocation of the wireless device structure
*  @param[in]   NONE
*  @return 	NONE
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
int WILC_WFI_DeInitHostInt(struct net_device *net)
{
	WILC_Sint32 s32Error = WILC_SUCCESS;
	struct WILC_WFI_priv *priv;
	perInterface_wlan_t *nic;
	
	priv = wdev_priv(net->ieee80211_ptr);
	nic = netdev_priv(net);
	WILC_SemaphoreDestroy(&(priv->hSemScanReq),NULL);

	priv->gbAutoRateAdjusted = WILC_FALSE;
	priv->bInP2PlistenState = WILC_FALSE;

	op_ifcs--;

	s32Error = host_int_deinit(priv->hWILCWFIDrv, net->name, nic->iftype);
	//s32Error = host_int_deinit(priv->hWILCWFIDrv_2);

	/* Clear the Shadow scan */
	clear_shadow_scan(priv);
	#ifdef DISABLE_PWRSAVE_AND_SCAN_DURING_IP
	if(op_ifcs==0)
	{
		PRINT_D(CORECONFIG_DBG, "destroy during ip\n");
		WILC_TimerDestroy(&hDuringIpTimer,WILC_NULL);
		del_timer_sync(&hEAPFrameBuffTimer);
	}
	#endif

	if(s32Error)
	{
		PRINT_ER("Error while deintializing host interface\n");
	}
	return s32Error;
}


/**
*  @brief 	WILC_WFI_WiphyFree
*  @details 	Freeing allocation of the wireless device structure
*  @param[in]   NONE
*  @return 	NONE
*  @author	mdaftedar
*  @date	01 MAR 2012
*  @version	1.0
*/
void WILC_WFI_WiphyFree(struct net_device *net)
{

	PRINT_D(CFG80211_DBG,"Unregistering wiphy\n");

	if(net == NULL){
		PRINT_D(INIT_DBG,"net_device is NULL\n");
		return;
		}

	if(net->ieee80211_ptr == NULL){
		PRINT_D(INIT_DBG,"ieee80211_ptr is NULL\n");
		return;
		}

	if(net->ieee80211_ptr->wiphy == NULL){
		PRINT_D(INIT_DBG,"wiphy is NULL\n");
		return;
		}

	wiphy_unregister(net->ieee80211_ptr->wiphy);

	PRINT_D(INIT_DBG,"Freeing wiphy\n");
	wiphy_free(net->ieee80211_ptr->wiphy);
	kfree(net->ieee80211_ptr);

}
