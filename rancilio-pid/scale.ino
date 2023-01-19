/********************************************************
 * Perfect Coffee PID
 * https://github.com/medlor/bleeding-edge-ranciliopid
 *****************************************************/
#include "scale.h"

bool getTareAsyncStatus() {
  return scaleTareSuccess;
}

#if (SCALE_SENSOR_ENABLE)
HX711_ADC LoadCell(SCALE_SENSOR_DATA_PIN, SCALE_SENSOR_CLK_PIN); 
#if (SCALE_SENSOR_2_ENABLE)
HX711_ADC LoadCell2(SCALE_SENSOR_2_DATA_PIN, SCALE_SENSOR_CLK_PIN); 
#endif

/********************************************************
   helper functions
******************************************************/

void scaleCalibration() {
  static unsigned long lastCalibration = 10000;
  if ( millis() >= lastCalibration + 2500) {
    lastCalibration = millis();

    if (!scaleRunning) {
      scalePowerUp();
      tareAsync();
    } else if (getTareAsyncStatus()) {
      float newCalibrationValue = LoadCell.getNewCalibration((float)SCALE_SENSOR_CALIBRATION_WEIGHT);
      DEBUG_print("Scale is tared to zero: You can now put your weight of %0.2fg on the scale: currentWeight=%0.2f. Calculated SCALE_SENSOR_CALIBRATION_FACTOR=%0.2f\n", (float)SCALE_SENSOR_CALIBRATION_WEIGHT, currentWeight, newCalibrationValue);   
      if ((currentWeight > 10) && (newCalibrationValue > 50) && (abs(LoadCell.getCalFactor() - newCalibrationValue) >= 5)) { 
        DEBUG_print("Scale calibration setting saved.\n");
        LoadCell.setNewCalibration((float)SCALE_SENSOR_CALIBRATION_WEIGHT);
        //TODO: save to eeprom and create "auto-calibration" ACTION
      }
      #if (SCALE_SENSOR_2_ENABLE)
      float newCalibrationValue2 = LoadCell2.getNewCalibration((float)SCALE_SENSOR_CALIBRATION_WEIGHT);
      DEBUG_print("Scale 2 is tared to zero: You can now put your weight of %0.2fg on the scale: currentWeight=%0.2f. Calculated SCALE_SENSOR_CALIBRATION_FACTOR=%0.2f\n", (float)SCALE_SENSOR_CALIBRATION_WEIGHT, currentWeight, newCalibrationValue2);   
      if ((currentWeight > 10) && (newCalibrationValue2 > 50) && (abs(LoadCell2.getCalFactor() - newCalibrationValue2) >= 5)) { 
        DEBUG_print("Scale 2 calibration setting saved.\n");
        LoadCell2.setNewCalibration((float)SCALE_SENSOR_CALIBRATION_WEIGHT);
      }
      #endif
    }
  }
}

void tareAsync() {
  LoadCell.tareNoDelay();
  LoadCell2.tareNoDelay();
  scaleTareSuccess = false;
  currentWeight = 0;  //reset to 0 when waiting for tare()
  currentWeight2 = 0;  //reset to 0 when waiting for tare()
  flowRate = 0.0;
  flowRateEndTime = millis() + 30000;
}

void scalePowerDown() {
  if (!scaleRunning) return;
  scaleRunning = false;
  LoadCell.powerDown();
  LoadCell2.powerDown();
}

void scalePowerUp() {
  scaleRunning = true;
  currentWeight = 0;
  LoadCell.powerUp();
  LoadCell2.powerUp();
}

void updateWeight() {
  if (!scaleRunning) return;
  //check if tareAsync() has triggered a tare() and read status of tare()
  if (!scaleTareSuccess && LoadCell.getTareStatus() && LoadCell2.getTareStatus()) {
        scaleTareSuccess = true;
        flowRateSampleTime = millis() - 100;
  }

  // get (smoothed) value from the dataset
  if (LoadCell.updateAsync() && LoadCell2.updateAsync()) {
    //only getData() when tare has completed
    if (scaleTareSuccess) {

      float previousWeight = currentWeight;
      currentWeight1 = LoadCell.getData();
      currentWeight2 = LoadCell2.getData();
      currentWeight = currentWeight1 + currentWeight2;
      DEBUG_print("Weight reading: cell1=%f, cell2=%f, total=%f\n", currentWeight1, currentWeight2, currentWeight);
      float diffWeight = currentWeight - previousWeight;
      // avoid negative wight increase
      if (diffWeight < 0)
        diffWeight = 0;
      float remainingWeight = *activeScaleSensorWeightSetPoint - scaleSensorWeightOffset - currentWeight ;

      unsigned long prevFlowRateSampleTime = flowRateSampleTime;
      flowRateSampleTime = millis();
      unsigned long diffFlowRateSampleTime = flowRateSampleTime - prevFlowRateSampleTime;
      if (diffFlowRateSampleTime > 110 || diffFlowRateSampleTime <= 11) {  //regular refresh on 10SPS every 90ms
        ERROR_print("flowRateSampleTime anomaly: %lums\n", diffFlowRateSampleTime);
        return;
      }

      float currentFlowRate = ( diffWeight * 1000.0 / diffFlowRateSampleTime) ;  //inaccuracy up tp 0.15g/s
      flowRate = (flowRate * (1-flowRateFactor) ) + (currentFlowRate * flowRateFactor); // smoothed gram/s
      if (flowRate < 0.1) flowRate = 0.1;  //just be sure to never have negative flowRate
      
      if (abs(flowRate) <= 0.1) {
        flowRateEndTime = millis() + 30000;  //during pre-infusion or after brew() we need special handling
      } else if (remainingWeight > 0) {
        int offsetTime = 0;
        flowRateEndTime = millis() + (unsigned long)((remainingWeight / flowRate) * 1000) - offsetTime; //in how many ms is the weight reached
      } else {
        flowRateEndTime = millis();  //weight already reached
      }

      DEBUG_print("updateWeight(%lums): weight=%0.3fg Diff=%0.2fg Offset=%0.2f currentFlowRate=%0.2fg/s flowRate=%0.2fg/s flowRateEndTime=%lums\n", 
        diffFlowRateSampleTime, currentWeight, diffWeight, scaleSensorWeightOffset, currentFlowRate, flowRate, (flowRateEndTime - millis()));
    }
  }

}

/********************************************************
   Initialize scale
******************************************************/
void IRAM_ATTR dataReadyISR(void *arg) {
  LoadCell.dataWaitingAsync();
#if (SCALE_SENSOR_2_ENABLE)
  LoadCell2.dataWaitingAsync();
#endif
}

static void attachISR_ESP32_SCALE(void *arg){					//attach ISR in freeRTOS because arduino can't attachInterrupt() inside of template class
  //DEBUG_print("attachISR_ESP32_2()\n");
  gpio_config_t gpioConfig;
  gpioConfig.pin_bit_mask = ((uint64_t)(((uint64_t)1)<<SCALE_SENSOR_DATA_PIN));
  gpioConfig.mode         = GPIO_MODE_INPUT;
  gpioConfig.pull_up_en   = GPIO_PULLUP_ENABLE ; //convertPortModeToDefine(_portMode) == INPUT_PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE; //convertPortModeToDefine(_portMode) == INPUT_PULLDOWN ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
  gpioConfig.intr_type    = GPIO_INTR_NEGEDGE;
  ESP_ERROR_CHECK(gpio_config(& gpioConfig));
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)SCALE_SENSOR_DATA_PIN, dataReadyISR, NULL);
  vTaskDelete(NULL);
}

void initScale() {
  initScaleModule(&LoadCell, 1, SCALE_SENSOR_CALIBRATION_FACTOR);
  initScaleModule(&LoadCell2, 2, SCALE_SENSOR_2_CALIBRATION_FACTOR);
}

void initScaleModule(HX711_ADC *load_cell, int cell_id, float cell_calibration_factor) {
  long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  for (int i=0; i<3;i++) {
    load_cell->begin();
    load_cell->start(stabilizingtime, _tare);
    //DEBUG_print("currentWeight: %0.2f (index=%d, getTareTimeoutFlag=%d, getSignalTimeoutFlag=%d)\n", currentWeight, LoadCell.getReadIndex(), LoadCell.getTareTimeoutFlag(), LoadCell.getSignalTimeoutFlag());
    if (load_cell->getTareTimeoutFlag() || load_cell->getSignalTimeoutFlag()) {
      ERROR_print("HX711 %d cannot be initialized (#%u)\n", cell_id, i);
      displaymessage(0, (char*)"Scale sensor defect", (char*)"");
    }
    else {
      DEBUG_print("HX711 %d initialized (#%u)\n", cell_id, i);
      load_cell->setCalFactor(cell_calibration_factor); // set calibration factor (float)
      #ifdef ESP32
      //CPU pinning does not have an effect (TSIC). Why?
      xTaskCreatePinnedToCore(attachISR_ESP32_SCALE,"attachISR_ESP32_SCALE",2000,NULL,1,NULL,1); //freeRTOS
      #else
      attachInterrupt(digitalPinToInterrupt(SCALE_SENSOR_DATA_PIN), dataReadyISR, FALLING);
      #endif
      break;
    }
    delay(200);
  }
}

#endif
