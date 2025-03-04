/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
//defines,directivas
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_queue.h"
#include "het.h"
#include "gio.h"
#include "sci.h"
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */

////////////////////////  VARIABLES Y FUNCIONES  ////////////////////////


int32_t x_cam; //Posicion en x
int32_t y_cam; //Posicion en y

int i=2; //Iteraciones para lazos

void vControl(void *PvParameters); //Tarea control
void sciNotification(sciBASE_t *sci, uint32 flags);

unsigned char ReceivedData[8]; //Arreglo de datos recibidos del sci

typedef struct {
    unsigned char inx3;
    unsigned char inx2;
    unsigned char inx1;
    unsigned char iny3;
    unsigned char iny2;
    unsigned char iny1;
}miDatSCI;
miDatSCI M_SCI;
xQueueHandle ColaSCI;

typedef struct {
    int32_t x_pos;
    int32_t y_pos;
}CAM_POS;

void pwmSetSignalMIO(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t signal);

static const uint32 s_het1pwmPolarity[8U] =
{
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
};
void limitControl(int32_t MIN, int32_t MAX, int32_t *x);

hetSIGNAL_t miPWMy, miPWMx;
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    gioInit();
    hetInit();
    sciInit();
    sciReceive(scilinREG,8,ReceivedData);

    ColaSCI = xQueueCreate(1,sizeof(miDatSCI));
    if (xTaskCreate(vControl, "Control", configMINIMAL_STACK_SIZE, NULL, 0, NULL) != pdTRUE){
        while(1);
    }//error
    vTaskStartScheduler();
    while(1);

/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
void vControl(void *PvParameters) //Tarea Control
{


    miDatSCI M_SCI_task;

// VALORES PRECONTROL
    float Kpx = 0.5;
    float Kdx = 3.5;
    int32_t Upx;
    int32_t Udx = 0;
    int32_t Ux=0;
    float Kpy = 0.5;
    float Kdy = 3.5;
    int32_t Upy;
    int32_t Udy = 0;
    int32_t Uy=0;
    CAM_POS ref_task;
    ref_task.x_pos = 0;
    ref_task.y_pos = 0;
    int32_t e_x[5] = {0};
    int32_t e_y[5] = {0};

//VALORES CONTROL
    float nKpx = 0.08; //Control X
    float nKdx = 0.005;
    float nKix = 0;
    int32_t nUpx;
    int32_t nUdx = 0;
    int32_t nUix=0;
    int32_t nUx=0;

    float nKpy = 0.06; //Control Y
    float nKdy = 0.01;
    float nKiy = 0;
    int32_t nUpy;
    int32_t nUdy = 0;
    int32_t nUiy = 0;
    int32_t nUy=0;

    CAM_POS nref_task;
    nref_task.x_pos = 175;
    nref_task.y_pos = 175;
    int32_t ne_x[5] = {0};
    int32_t ne_y[5] = {0};



    for(;;)
    {
        xQueueReceive(ColaSCI, &M_SCI_task,20/portTICK_RATE_MS);

        x_cam =(int32_t)((M_SCI_task.inx1 - '0') + 10*(M_SCI_task.inx2 - '0') + 100*(M_SCI_task.inx3 - '0'));
        y_cam =(int32_t)((M_SCI_task.iny1 - '0') + 10*(M_SCI_task.iny2 - '0') + 100*(M_SCI_task.iny3 - '0'));

        if(i%2==0){//PARA QUE SE USE EL CONTROL CADA 40ms

            ne_x[4] = ne_x[3];
            ne_x[3] = ne_x[2];
            ne_x[2] = ne_x[1];
            ne_x[1] = ne_x[0];
            ne_x[0] = x_cam - nref_task.x_pos;

            ne_y[4] = ne_y[3];
            ne_y[3] = ne_y[2];
            ne_y[2] = ne_y[1];
            ne_y[1] = ne_y[0];
            ne_y[0] = y_cam - nref_task.y_pos;

            nUpx = (int32_t)(nKpx*ne_x[0]);
            nUdx = (int32_t)(nKdx*(ne_x[0]-ne_x[4]));
            nUix=nUix+(int32_t)(nKix*ne_x[0]);
            nUx = nUpx + nUdx + nUix;

            limitControl(-100, 100, &nUx);
            nUpy = (int32_t)(nKpy*ne_y[0]);
            nUdy = (int32_t)(nKdy*(ne_y[0]-ne_y[4]));
            nUiy=nUiy+nKiy*ne_y[0];
            nUy= nUpy + nUdy + nUiy;
            limitControl(-100, 100, &nUy);

            ref_task.x_pos= nref_task.x_pos - nUx;
            ref_task.y_pos= nref_task.y_pos - nUy;
        }

        e_x[4] = e_x[3];
        e_x[3] = e_x[2];
        e_x[2] = e_x[1];
        e_x[1] = e_x[0];
        e_x[0] = x_cam - ref_task.x_pos;
        e_y[4] = e_y[3];
        e_y[3] = e_y[2];
        e_y[2] = e_y[1];
        e_y[1] = e_y[0];
        e_y[0] = y_cam - ref_task.y_pos;

        Upx = (int32_t)(Kpx*e_x[0]);
        Udx = (int32_t)(Kdx*(e_x[0]-e_x[4]));
        Ux = Upx + Udx;
        limitControl(-100, 100, &Ux);

        Upy = (int32_t)(Kpy*e_y[0]);
        Udy = (int32_t)(Kdy*(e_y[0]-e_y[4]));
        Uy = Upy + Udy;
        limitControl(-100, 100, &Uy);

        miPWMx.duty = 767 - Ux;
        miPWMx.period = 20000;
        pwmSetSignalMIO(hetRAM1, pwm1, miPWMx);

        miPWMy.duty = 795 + Uy;
        miPWMy.period = 20000;
        pwmSetSignalMIO(hetRAM1, pwm0, miPWMy);

       /* if(i%250==0){  //Pruebas con cambio de referencia
            switch (ref_task.x_pos){
                case 100:
                    ref_task.x_pos = 250;
                    ref_task.y_pos = 250;
                    break;
                case 250:
                    ref_task.x_pos = 100;
                    ref_task.y_pos = 100;
                    break;
            }
        }*/
        i++;

    }
}

void sciNotification(sciBASE_t *sci, uint32 flags)
{
    miDatSCI M_SCI_interr;
    BaseType_t xCoRoutinePreviouslyWoken=0;
    sciReceive(scilinREG,8,ReceivedData);
    M_SCI_interr.inx3 =  ReceivedData[1];
    M_SCI_interr.inx2 =  ReceivedData[2];
    M_SCI_interr.inx1 =  ReceivedData[3];
    M_SCI_interr.iny3 =  ReceivedData[5];
    M_SCI_interr.iny2 =  ReceivedData[6];
    M_SCI_interr.iny1 =  ReceivedData[7];
    xQueueSendFromISR(ColaSCI,&M_SCI_interr,xCoRoutinePreviouslyWoken );
}

void limitControl(int32_t MIN, int32_t MAX, int32_t *x)
{
    int32_t *aux = x;
    if(*aux < MIN)
    {
        *aux = MIN;
    }
    if(*aux > MAX)
    {
        *aux = MAX;
    }
}

void pwmSetSignalMIO(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t signal)
{
    uint32 action;
    uint32 pwmPolarity = 0U;
    float64 pwmPeriod = 0.0F;

    if(hetRAM == hetRAM1)
    {
        pwmPeriod = (signal.period * 1000.0F) / 640.000F;
        pwmPolarity = s_het1pwmPolarity[pwm];
    }
    else
    {
    }
    if (signal.duty == 0U)
    {
        action = (pwmPolarity == 3U) ? 0U : 2U;
    }
    else if (signal.duty >= 10000U)//modificamos el 10000
    {
        action = (pwmPolarity == 3U) ? 2U : 0U;
    }
    else
    {
        action = pwmPolarity;
    }

    hetRAM->Instruction[(pwm << 1U) + 41U].Control = ((hetRAM->Instruction[(pwm << 1U) + 41U].Control) & (~(uint32)(0x00000018U))) | (action << 3U);
    hetRAM->Instruction[(pwm << 1U) + 41U].Data = ((((uint32)pwmPeriod * signal.duty) / 10000U) << 7U ) + 128U;//se modifica el 10000
    hetRAM->Instruction[(pwm << 1U) + 42U].Data = ((uint32)pwmPeriod << 7U) - 128U;

}

/* USER CODE END */

