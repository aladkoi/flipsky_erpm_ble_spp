#include "controller.h"
//#include "esp_log.h"
#include "crc.h"


// Инициализация состояния
ControllerState_t state = {
    //.name_cont = "FT85BS",
    .name_cont = "",
    .Vbatt = 0.0f,
    .rpm_controller = 0,
    .volt_bl = 0,
    //.break_volt_bl = 0,
    .current_rpm = 0,
    .current_level = 0,
    .current_level_speed = 0,
    .current_level_rpm = 0,
    .croiuse_level = -1,
    //.break_croiuse_level = -1,
    .controllerBrake = false,
    .currentSpeed = 0,
    .cr = -1,
    .current_amper = 3, // Начальный индекс шага скорости
    .target_erpm = 0,
    .crouise_on = false,
    .isn = true,
    .count_telemtr = 0,
    .numberCrouise = 255,
    .stop_from_mobile = false,
    .break_long = false,
    .speed_up = false,
    .addspeed = false,
    .break_level = 1,
    .operation = 0,
    .change_event=0,
    .volt_add_speed=0,
    .start_break_event=false,
    .limit_speed=false,
    .auto_speed=true,
    .start_level=7,
    .auto_start_level=7,
    .smart_brake=true,
    .diagnostic=true
};

volatile float level_crouise[6] = {0, 21.4, 33.2, 46.0, 58.4, 95.0};
volatile int rpm_crouise[6] = {2611, 4700, 7330, 9699, 12870, 20800};
volatile int len_crouise = 5; // Размер массива круиза минус 1


SemaphoreHandle_t state_mutex = NULL;

void controller_init(void) {
    if (state_mutex == NULL) {
        state_mutex = xSemaphoreCreateMutex();
        if (state_mutex == NULL) {
            //printf("Failed to create state mutex");
        } else {
            //printf("State mutex initialized");
        }
    }
}

// Функции управления состоянием
void stop_Speed(bool status) {
    if (status && !state.controllerBrake) {
        state.controllerBrake = true;
        if (!state.isn) { // Умный тормоз
            state.current_level = 0;
            state.current_rpm = 0;
        }
        state.volt_bl = 0;
        state.numberCrouise = 255;
        state.crouise_on = false;
        state.croiuse_level = -1;
        state.cr = -1;
        //getDuty(0.0);
}
}

void setCurrentLevel(void) {
    //printf("setCurrentLevel\n");
    state.current_level_speed = state.volt_bl;
    state.current_level = state.volt_bl;
    state.current_level_rpm = state.rpm_controller;
}

int get_level(bool forward) {
int cr = -1;
    if (forward && state.volt_bl >= rpm_crouise[len_crouise]) {
        cr = len_crouise;
    } else if (forward) {
        for (int i = 0; i <= len_crouise; i++) {
            if (rpm_crouise[i] > state.volt_bl) {
                cr = i;
                break;
            }
        }
    } else {
        for (int i = len_crouise; i >= 0; i--) {
            if (state.volt_bl > rpm_crouise[i]) {
                cr = i;
                break;
            }
        }
    }
    if (!forward && cr == -1) cr = -1;
    else if (cr == -1) cr = 0;
return cr;
}

void select_level(int crouise) {
    if (state.volt_bl == 0 || (crouise > -1 && crouise < (len_crouise + 1))) {
        state.volt_bl = rpm_crouise[crouise];
      if (!state.speed_up) {
          state.currentSpeed = rpm_crouise[crouise];
      } else {          
          if (crouise > 0 && state.auto_speed) {
              state.addspeed = true;
          }
      }
      state.controllerBrake = false;
  }  
}

int start_crouise(void) {
printf("start_crouise=%d\n",state.start_level);    
state.volt_bl=erpm_step[state.start_level];
state.croiuse_level = get_level(true);
return state.volt_bl;
// int result = rpm_crouise[state.start_level];
//   state.croiuse_level = 0;
//   select_level(state.croiuse_level);
//   if (state.volt_bl <= erpm_step[state.start_level]) state.volt_bl = rpm_crouise[state.start_level];
//   if (state.volt_bl > rpm_crouise[len_crouise]) state.volt_bl = rpm_crouise[len_crouise];
//   //getDuty(state.volt_bl);
//   state.current_level = state.volt_bl;
//   result = state.volt_bl;
//   printf("return start_crouise=%d\n",result);    
// return result;
}


void AddSpeed(void) {
  printf("AddSpeed\n");
  printf("state.volt_bl=%d\n",state.volt_bl);
  printf("rpm_crouise[len_crouise]=%d\n",rpm_crouise[len_crouise]);
  if (state.break_level == 1 && state.volt_bl < rpm_crouise[len_crouise]) {
      if (state.volt_bl < erpm_step[state.start_level] || state.volt_bl == 0) {
          state.volt_bl = start_crouise();
      } else {
          printf("getStep\n");
          state.volt_bl = getStep(true, state.volt_bl);
          printf("state.volt_bl=%d\n",state.volt_bl);       
          if (state.volt_bl > rpm_crouise[len_crouise]) state.volt_bl = rpm_crouise[len_crouise];
      }
  }
  //printf("AddSpeed1\n");
  setCurrentLevel();
}

void setCrouise(int crouise) {
 if (state.cr == crouise) {            
      return;
  }
  state.crouise_on = true;
  state.cr = crouise;
  state.numberCrouise = crouise;
  select_level(crouise);
  //printf("setCrouise1\n");
  setCurrentLevel();
}


void BUTTON_PRESS_REPEAT_ADD(void){
    state.cr=-1;
    if (state.croiuse_level < len_crouise) {
        state.croiuse_level = get_level(true);
        if (state.croiuse_level < 1) state.croiuse_level = 0;
        if (state.croiuse_level == 0) state.croiuse_level = 1;
        state.crouise_on = true;
        state.speed_up = true;
        setCrouise(state.croiuse_level);
        state.change_event=state.croiuse_level+1;
    }
}

void BUTTON_SINGLE_CLICK_ADD(void){
    state.break_level=1;
    if (state.croiuse_level == -1 && state.rpm_controller == 0) {        
        printf("setCrouise()%d=\n",state.croiuse_level);
        state.volt_bl=erpm_step[state.start_level];
        state.croiuse_level = get_level(true);
        // state.croiuse_level = 0;
        state.cr=-1;
        state.crouise_on = true;
        state.speed_up = true;
        state.change_event=1;
        printf("state.volt_bl=%d\n",state.volt_bl);
        printf("state.croiuse_level%d=\n",state.croiuse_level);
        //setCrouise(state.croiuse_level);
        //start_crouise();
    } else {
        printf("do AddSpeed()");
        AddSpeed();
        state.currentSpeed = 0;
    }
}

void BUTTON_SINGLE_CLICK_DEC(void){
    state.crouise_on = false;
    if (state.break_level == 1 && state.volt_bl >= state.start_level)
        state.volt_bl = getStep(false, state.volt_bl);
    if (state.volt_bl < 0) {
        state.volt_bl = 0;
        state.change_event=20;
    }
    setCurrentLevel();
    printf("======state.volt_bl=%d\n",state.volt_bl);
}

void BUTTON_PRESS_REPEAT_DEC(void){
    state.croiuse_level = get_level(false);
    state.change_event=state.croiuse_level+1;
    if (state.croiuse_level < 0) {
        state.volt_bl = 0;
        setCurrentLevel();
        state.change_event=20;
    }
    if (state.croiuse_level > len_crouise) state.croiuse_level = len_crouise;
    if (state.croiuse_level != -1) {
        state.crouise_on = true;
        state.speed_up = false;
        setCrouise(state.croiuse_level);
    } else {
        state.stop_from_mobile = true;
    }
}