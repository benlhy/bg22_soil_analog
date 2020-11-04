#include "app.h"

#include "em_cmu.h"
#include "em_common.h"
#include "em_i2c.h"
#include "em_iadc.h"
#include "gatt_db.h"
#include "sl_app_assert.h"
#include "sl_app_log.h"
#include "sl_bluetooth.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_led_instances.h"
#include "sl_simple_timer.h"
#include "sl_status.h"
#include "ustimer.h"

// Timer
#define READ_PERIOD 2000

// Set CLK_ADC to 10MHz (this corresponds to a sample rate of 77K with OSR = 32)
// CLK_SRC_ADC; largest division is by 4
#define CLK_SRC_ADC_FREQ 20000000

// CLK_ADC; IADC_SCHEDx PRESCALE has 10 valid bits
#define CLK_ADC_FREQ 10000000

// When changing GPIO port/pins above, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_BUS CDBUSALLOC
#define IADC_INPUT_BUSALLOC GPIO_CDBUSALLOC_CDODD0_ADC0

// Stores latest ADC sample and converts to volts
static volatile IADC_Result_t sample;
static volatile double singleResult;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

static void usr_read_soil_sensor(void);

static sl_simple_timer_t read_sensor_timer;

void initIADC(void) {
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Enable IADC clock
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  // Configure IADC clock source for use while in EM2
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

  // Modify init structs and initialize
  init.warmup = iadcWarmupKeepWarm;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  // Configuration 0 is used by both scan and single conversions by default
  // Use unbuffered AVDD as reference
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency for desired sample rate
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(
      IADC0, CLK_ADC_FREQ, 0, iadcCfgModeNormal, init.srcClkPrescale);

  // Set oversampling rate to 32x
  // resolution formula res = 11 + log2(oversampling * digital averaging)
  // in this case res = 11 + log2(32*1) = 16
  // initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;

  // Single initialization
  // initSingle.dataValidLevel = _IADC_SINGLEFIFOCFG_DVL_VALID1;

  // Set conversions to run continuously
  // initSingle.triggerAction = iadcTriggerActionContinuous;
  initSingle.triggerAction = iadcTriggerActionOnce;

  // Set alignment to right justified with 16 bits for data field
  initSingle.alignment = iadcAlignRight12;

  // Configure Input sources for single ended conversion
  initSingleInput.posInput = iadcPosInputPortCPin7;
  initSingleInput.negInput = iadcNegInputGnd;

  // Initialize IADC
  // Note oversampling and digital averaging will affect the offset correction
  // This is taken care of in the IADC_init() function in the emlib
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize Scan
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_BUS |= IADC_INPUT_BUSALLOC;

  // Enable interrupts on data valid level
  IADC_enableInt(IADC0, IADC_IEN_SINGLEDONE);
  // IADC_enableInt(IADC0, IADC_IEN_SINGLEFIFODVL);

  // Enable ADC interrupts
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}

void IADC_IRQHandler(void) {
  // Read data from the FIFO, 16-bit result
  sample = IADC_readSingleResult(IADC0);
  // sample = IADC_pullSingleFifoResult(IADC0);

  // For single-ended the result range is 0 to +Vref, i.e., 16 bits for the
  // conversion value.
  singleResult = sample.data * 3.0 / 0xFFF;
  sl_app_log("ADC val: %f \n", singleResult);

  IADC_clearInt(IADC0, IADC_IEN_SINGLEDONE);
  // IADC_clearInt(IADC0, IADC_IF_SINGLEFIFODVL);
}

SL_WEAK void app_init(void) {
  sl_app_log("APP initialised\n");
  // sl_simple_button_init_instances(); -> already init in main.c
  // sl_simple_led_init_instances();
  USTIMER_Init();
  initIADC();

  sl_status_t sc;

  // This function starts the usr_read_soil_sensor function and is triggered
  // every READ_PERIOD period (ms)
  sc = sl_simple_timer_start(&read_sensor_timer, READ_PERIOD,
                             usr_read_soil_sensor, NULL, true);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to start sensor timer\n", (int)sc);

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

SL_WEAK void app_process_action(void) {
  sl_simple_button_poll_instances();
  sl_button_state_t button_state = sl_button_get_state(&sl_button_btn0);

  if (button_state == SL_SIMPLE_BUTTON_PRESSED) {
    IADC_command(IADC0, iadcCmdStartSingle);
    // USTIMER_Delay(250000);
    // sample = IADC_readSingleResult(IADC0);
    // singleResult = sample.data * 3.3 / 0xFFFF;
    // sl_app_log("ADC val: %lf \n",singleResult);

    sl_app_log("Button is pressed!\n");
    sl_led_toggle(&sl_led_led0);

    USTIMER_Delay(250000);
  }
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************/
/**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt) {
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\n", (int)sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(
          gattdb_system_id, 0, sizeof(system_id), system_id);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\n", (int)sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to create advertising set\n", (int)sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
          advertising_set_handle,
          160,  // min. adv. interval (milliseconds * 1.6)
          320,  // max. adv. interval (milliseconds * 1.6)
          0,    // adv. duration
          0);   // max. num. adv. events
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set advertising timing\n", (int)sc);
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(advertising_set_handle,
                                  advertiser_general_discoverable,
                                  advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n", (int)sc);

      sl_bt_sm_set_bondable_mode(1);
      // sl_bt_sm_configure();
      // sl_bt_sm_store_bonding_configuration
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      // upgrade the security.
      // sl_bt_sm_increase_security(evt->data.evt_connection_opened.connection);
      sl_app_log("Connected!");

      break;

    case sl_bt_evt_sm_bonded_id:
      sl_app_log("Bonded!\n");
      break;

    case sl_bt_evt_sm_bonding_failed_id:
      sl_app_log((int)(evt->data.evt_sm_bonding_failed.reason));

      sl_bt_sm_delete_bondings();
      sl_bt_connection_close(evt->data.evt_sm_bonding_failed.connection);
      sl_app_log("Bonding failed.\n");

      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Restart advertising after client has disconnected.
      sl_app_log("Disconnected!");
      sc = sl_bt_advertiser_start(advertising_set_handle,
                                  advertiser_general_discoverable,
                                  advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n", (int)sc);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

// --------------------------------------
static void usr_read_soil_sensor() {
  IADC_command(IADC0, iadcCmdStartSingle);
  USTIMER_Delay(250000);

  sl_app_log("ADC: %d \n", sample.data);

  sl_bt_gatt_server_send_characteristic_notification(0xFF, gattdb_analog, 2,
                                                     &sample.data, NULL);
}
