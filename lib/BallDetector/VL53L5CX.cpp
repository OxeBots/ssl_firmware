
// include lib 
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <Vl53l5cx_api.h>


uint8_t             status, loop, isAlive, isReady, i;
VL53L5CX_config     Dev; 
VL53L5CX_result     Results; 


// need i2c config 


// Power on range basics

    //check if o sensor tá vivo 
status = vl53l5cx_is_alive(&Dev, &isAlive);
if(!isAlive || status)
{
    printf("VL53L5CS is not requested\n");
    return;
}

//init sensor 

status = vl53l5cx_init(&Dev); 
if(status)
{
    printf("VL53L5CX ULD is failed\n");
    return;
}

printf("VL53L5CX is ready!\n"); 

loop = 0; 

while(loop < 10)
{
    status = vl53l5cx_check_data_ready(&Dev, &isReady); 

    if(isReady)
    {
        vl53l5cx_get_ranging_data(&Dev, &Results)

        printf("Print data no: %3u\n", Dev.streamcount())
        for( i = 0; i < 16; i++)
        {
            printf("Zone : %3d, Status: %3u, Distance : %4d mm\n",
                i, 
                Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
                Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);
                
        }
        printf("\n");
        loop++; 
    }

    VL53L5CX_Waitms(&(Dev.platform), 5);
}

status = vl53l5cx_stop_ranging(&Dev);
printf("End basic range mode");