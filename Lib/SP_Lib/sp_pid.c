/**
  ******************************************************************************
  * @file       template.c
  * @author     YTom/LSS
  * @version    v0.0-alpha
  * @date       2018.Nov.21
  * @brief      project source file template
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_pid.h"
#include "main.h"


void PID_ControllerInit(PID_Type* pid, uint16_t lim_i, uint16_t lim_d, uint16_t lim_out, float dt1) {
    memset(pid, 0x00, sizeof(PID_Type));
    pid->intergration_limit = lim_i;
    pid->differential_limit = lim_d;
    if(lim_out==(uint16_t)-1) {
        pid->ctrl_reg.using_output_limit = false;
    } else {
        pid->output_limit = lim_out;
        pid->ctrl_reg.using_output_limit = true;
    }
    pid->dT = dt1;
    pid->ctrl_reg.using_delta_limit = false;
    pid->ctrl_reg.initialized = true;
    pid->sum_error = 0;
    pid->intergration_separation = pid->intergration_limit*2;
}

float PID_ControllerDriver(PID_Type* pid, float target, float input) {
    // uint8_t i;
    volatile float pterm, iterm, dterm;
    float error;
    float trend_input=0, trend_error=0, trend_output=0;
    
    /*------------------------------PART I-------------------------------*/
    if(fabs(pid->target - target) > 0.01f)
        PID_SetTarget(pid, target);
//    if(pid->target_changed)
//    //TODO:
    
    /* Prepend input and calculate its trend(slope) */
    trend_input = (input - pid->inputs[2])/3;       // Average delta inputs (slope)
    if(pid->functions.input_filter) {
        input = pid->functions.input_filter(pid->inputs, sizeof(pid->inputs)/sizeof(pid->inputs[0]),
            input, (uint16_t)-1);
    } else {
        pid->inputs[2] = pid->inputs[1];
        pid->inputs[1] = pid->inputs[0];
        pid->inputs[0] = input;
    }
        
    /* Update error */
    error = pid->target - input;
//    /* Using moving average */
//    //TODO: optimize 10.f
//    if(fabs(pid->errors[0] - pid->errors[1])>10.f &&
//        fabs(error - pid->errors[0])/fabs(pid->errors[0] - pid->errors[1]) > PID_CHANGERATE_THRESHOLD) {
//        //TODO: if it's useful
//        error = (1.6f*error + 1.2f*pid->errors[0] + 0.8f*pid->errors[1] + 0.4f*pid->errors[2])/4;
//    }
//    trend_error = (fabs(error)-fabs(pid->errors[2]))/3;     // Absolute average delta error
    pid->errors[2] = pid->errors[1];
    pid->errors[1] = pid->errors[0];
    pid->errors[0] = error;
    
    /*------------------------------PART II------------------------------*/
    /* Calc calssical incremental PID, result delta U[k] */
    pterm = pid->Kp * pid->errors[0];
    /* Integral separation */
    if(fabs(pid->errors[0]) < pid->intergration_separation) {
        /* Using TRAPEZOID intergration instead of RECTANGLE intergration */
//        pid->sum_error += error * pid->dT;      //----RECTANGLE intergration
        pid->sum_error += (pid->errors[0] + pid->errors[1]) * pid->dT / 2;      //----TRAPEZOID intergration
        /* Intergration limit */
        if(fabs(pid->sum_error) > pid->intergration_limit) {
            pid->sum_error = (pid->sum_error>0)? pid->intergration_limit:-pid->intergration_limit;
        } 
        iterm = pid->Ki * pid->sum_error;
    } else {
        iterm = 0;
    }
    /* Differential limit */
    dterm = pid->Kd * (pid->errors[0] - pid->errors[1]) / pid->dT;
    if(fabs(dterm) > pid->differential_limit) {
        dterm = (dterm>0)? pid->differential_limit:-pid->differential_limit;
    }
    /* Output delta U[k] */
    float new_output = pterm + iterm + dterm;
//    if(fabs(delta_uk) > pid->output_limit) {
//        delta_uk = (delta_uk>0)? output_limit
//    }
//    /* Update delta */
//    size = sizeof(pid->deltas)/sizeof(pid->deltas[0]);
//    for(i=size-1; i>0; i--) {
//        // avg_output += pid->deltas[i];
//        pid->deltas[i] = pid->deltas[i-1];
//    }
//    pid->deltas[0] = delta_uk;
    
    /*------------------------------PART III-----------------------------*/
    /* Using bang-bang control when output is big enough */
    if(fabs(new_output) > (pid->output_limit*PID_BANGBANG_THRESHOLD)) {
        new_output = (new_output>0)?pid->output_limit:-pid->output_limit;
    } else if(fabs(new_output) < (pid->output_limit*PID_SOFTPID_THRESHOLD) && !pid->functions.output_filter) {
        // TOOD: soft smooth control
        new_output = (new_output + pid->ouputs[0] + pid->ouputs[1] + pid->ouputs[2])/4;
    }
    /* Update output */
    trend_output = (new_output - pid->ouputs[2])/3;    // Average delta inputs (slope)
    if(pid->functions.output_filter) {
        new_output = pid->functions.output_filter(pid->ouputs, sizeof(pid->ouputs)/sizeof(pid->ouputs[0]),
            new_output, (uint16_t)-1);
    } else {
        pid->ouputs[2] = pid->ouputs[1];
        pid->ouputs[1] = pid->ouputs[0];
        pid->ouputs[0] = new_output;
    }
    
    return new_output;
}


float PID_ControllerDriver_Incremental(PID_Type* pid, float target, float input, uint16_t lim_delta) {
    // uint8_t i;
    volatile float pterm, iterm, dterm;
    float error;
    
    /*------------------------------PART I-------------------------------*/
    if(fabs(pid->target - target) > 1.0f)
        PID_SetTarget(pid, target);
    
    /* Update input */
    pid->inputs[2] = pid->inputs[1];
    pid->inputs[1] = pid->inputs[0];
    pid->inputs[0] = input;
    
    /* Update error */
    error = pid->target - input;
    /* Using moving average */
    //TODO: optimize 10.f
//    if(fabs(error - pid->errors[0])/fabs(pid->errors[0]) > 2.0f) {
//        //TODO: if it's useful
//        /* Error increses rapidly will be regarded as abnormal*/
//        error = (0.8f*error + 1.2f*pid->errors[0] + 1.0f*pid->errors[1] + 1.0f*pid->errors[2])/4;
//    }
    pid->errors[2] = pid->errors[1];
    pid->errors[1] = pid->errors[0];
    pid->errors[0] = error;
    
    /*------------------------------PART II------------------------------*/
    /* Calc calssical incremental PID, result delta U[k] */
    pterm = pid->Kp * (pid->errors[0] - pid->errors[1]);
    /* Integral separation */
    if(fabs(pid->errors[0]) < pid->intergration_separation) {
        /* Using TRAPEZOID intergration */
        iterm = pid->Ki * pid->dT * (pid->errors[0] + pid->errors[1])/2;
    } else {
        iterm = 0;
    }
    /* Differential limit */
    dterm = pid->Kd * (pid->errors[0] - 2*pid->errors[1] + pid->errors[2]) / pid->dT;
    /* Output delta U[k] */
    float new_output = pterm + iterm + dterm;
//    /* Output limit */
//    if(fabs(new_output) > pid->output_limit) {
//        new_output = (new_output>0)?pid->output_limit:-pid->output_limit;
//    }
    
    /*------------------------------PART III-----------------------------*/
    /* Using bang-bang control when output is big enough */
    if(fabs(new_output) > (lim_delta*PID_BANGBANG_THRESHOLD)) {
        new_output = (new_output>0)?lim_delta:-lim_delta;
    }else if(fabs(new_output) < (lim_delta*PID_SOFTPID_THRESHOLD)) {
        // TOOD: soft smooth control
        new_output = (1.6f*(new_output + pid->ouputs[0]) +
            1.2f*pid->ouputs[0] + 
            0.8f*pid->ouputs[1] + 
            0.4f*pid->ouputs[2])/4;
    }
    /* Update output */
    pid->ouputs[2] = pid->ouputs[1];
    pid->ouputs[1] = pid->ouputs[0];
    pid->ouputs[0] += new_output;
    /* Output limit */
    if(fabs(pid->ouputs[0]) > pid->output_limit) {
        pid->ouputs[0] = (pid->ouputs[0]>0)?pid->output_limit:-pid->output_limit;
    }
    
    return pid->ouputs[0];
}


float PID_ControllerDriver_test(PID_Type* pid, float target, float input) {
    // uint8_t i;
    volatile float pterm, iterm, dterm;
    float error;
    
    /*------------------------------PART I-------------------------------*/
    if(fabs(pid->target - target) > 0.01f)
        PID_SetTarget(pid, target);
    
    /* Update error */
    error = pid->target - input;
    pid->errors[2] = pid->errors[1];
    pid->errors[1] = pid->errors[0];
    pid->errors[0] = error;
        
    /*------------------------------PART II------------------------------*/
    /* Calc calssical incremental PID, result delta U[k] */
    pterm = pid->Kp * pid->errors[0];
    /* Integral separation */
    /* Using TRAPEZOID intergration instead of RECTANGLE intergration */
    pid->sum_error += (pid->errors[0] + pid->errors[1]) * pid->dT / 2;      //----TRAPEZOID intergration
    if(fabs(pid->sum_error) > pid->intergration_limit) {
        pid->sum_error = (pid->sum_error>0)? pid->intergration_limit:-pid->intergration_limit;
    }
    /* Intergration limit */
    iterm = pid->Ki * pid->sum_error;

    /* Differential limit */
    dterm = pid->Kd * (pid->errors[0] - pid->errors[1]) / pid->dT;
    /* Output delta U[k] */
    float new_output = pterm + iterm + dterm;

    return new_output;
}




void PID_SetTarget(PID_Type* pid, float target) {
    pid->target = target;
    pid->ctrl_reg.target_changed = 1;
}

void PID_SetGains(PID_Type* pid, float kp, float ki, float kd) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

void PID_UpdateLimits(PID_Type* pid, uint16_t lim_i, uint16_t lim_d, uint16_t lim_out) {
    pid->intergration_limit = lim_i;
    pid->differential_limit = lim_d;
    pid->output_limit = lim_out;
}


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
