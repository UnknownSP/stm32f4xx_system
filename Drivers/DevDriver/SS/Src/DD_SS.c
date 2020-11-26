/* ===Kisarazu RBKN Library===
 *
 * autor          : Idutsu
 * version        : v0.10
 * last update    : 20180501
 *
 * **overview***
 * SSの通信プロトコルを定める。
 *
 * ・I2Cのみのサポート
 */
#include <stdint.h>
#include <stdbool.h>
#include "message.h"
#include "DD_SS.h"
#include "DD_Gene.h"
#include "app.h"
/*
 *2018年第31回ロボコンでの自動ロボット自立制御の為追加
 *deviceDefinition.cで使用するセンサドライバの数、初期設定を行い
 *使用する
 */

static int ReadPoint  = 0;
static int WritePoint = 0;
static int *RingBuf[RINGSIZE];
static uint8_t temp_data;
static DD_SSCom_Phase phase = TRANSMIT_PHASE;

static bool error_flag = false;

static int encoder_wait[8] = {0,0,0,0,0,0,0,0};

/*SystemTaskManagerにより繰り返し実行される*/
int DD_receive2SS(void){
  /*I2Cのアドレス、受信データを格納する配列とそのサイズを
   *受信用の関数に引数として渡し、返り値として成功か否かを 
   *返す
   *引数として渡した配列に受信データが格納される
   *引数として渡す配列は、アドレスを渡す
   */
  int ret = 0;

#if DD_NUM_OF_SS

  if(error_flag){
    return -1;
  }
  
  if(phase == TRANSMIT_PHASE){
    if(Empty_Check()){
      return 0;
    }else{
      DD_SSPullReceiveRequest(&temp_data);
    }
  }

  switch(g_ss_h[temp_data].type){
  case D_STYP_ENCODER:
  case D_STYP_ODMETRY:
    encoder_wait[temp_data]++;
    if(phase == TRANSMIT_PHASE){
      if(encoder_wait[temp_data] >= 1){
	      encoder_wait[temp_data] = 0;
	      ret = Read_ENCODER(&g_ss_h[temp_data]);
	      if(ret){
	        //error_flag = true;
	        return ret;
	      }
      }
    }
    break;
  }

#endif
  
  return ret;
}


/*
 * *SS handlerを表示。
 *
 * SS(Add:hex):[データサイズ],[受信データ]*データサイズ分
 */

void DD_SSHandPrint(DD_SSHand_t *dmd){
  MW_printf("SS(%02x):[%d]", dmd->add,dmd->data_size);
  //if(dmd->type == D_STYP_PHOTOARRAY){
  //  for(int i=0;i<dmd->data_size;i++){
  //    MW_printf("[%08b]",dmd->data[i]);
  //  }
  //}else 
  if(dmd->type == D_STYP_ODMETRY){
    //uint32_t E1 = (dmd->data[0])+(dmd->data[1])*256+(dmd->data[2])*65536+(dmd->data[3])*16777216; 
    //uint32_t E2 = (dmd->data[4])+(dmd->data[5])*256+(dmd->data[6])*65536+(dmd->data[7])*16777216; 
    int32_t x,y,w;

    x = dmd->data[2] + (dmd->data[3])*256 + ( (0b01111111)&(dmd->data[4]) ) *65536;
    if(( ((0b10000000)&(dmd->data[4]))>>7 ) == 1){
      x *= -1;
    }
    y = dmd->data[5] + (dmd->data[6])*256 + ( (0b01111111)&(dmd->data[7]) ) *65536;
    if(( ((0b10000000)&(dmd->data[7]))>>7 ) == 1){
      y *= -1;
    }
    w = dmd->data[0] + ( (0b01111111)&(dmd->data[1]) ) * 256;
    if(( ((0b10000000)&(dmd->data[1]))>>7 ) == 1){
      w *= -1;
    }

    //MW_printf("\nposition[x][y][w] : [%10d][%10d][%6d]\n",x,y,w);  
    for(int i=0;i<dmd->data_size;i++){ 
      MW_printf("[%4d]",dmd->data[i]); 
    }
    //MW_printf("\n[E1]:[%10d] [E2]:[%10d]",E1,E2);
  }else if(dmd->type == D_STYP_ENCODER){
    //uint32_t E1 = (dmd->data[0])+(dmd->data[1])*256+(dmd->data[2])*65536+(dmd->data[3])*16777216; 
    //uint32_t E2 = (dmd->data[4])+(dmd->data[5])*256+(dmd->data[6])*65536+(dmd->data[7])*16777216; 
    for(int i=0;i<dmd->data_size;i++){ 
      MW_printf("[%4d]",dmd->data[i]); 
    } 
    //MW_printf("\n[E1]:[%10d] [E2]:[%10d]",E1,E2);
  }else{
    for(int i=0;i<dmd->data_size;i++){
      MW_printf("[%02x]",dmd->data[i]);
    }
  }
  MW_printf("\n");
}

int SS_Init(DD_SSHand_t *dmd){

  switch(dmd->type){
  case D_STYP_ENCODER:
  case D_STYP_ODMETRY:
    break;
  }
  return 0;
}

int DD_SSPutReceiveRequest(uint8_t num){
  int next = (WritePoint + 1) % RINGSIZE;
  if(next == ReadPoint){
    return -1;
  }
  RingBuf[WritePoint] = num;
  //RingBuf[WritePoint] = num;
  WritePoint = next;
  return 0;
}

int DD_SSPullReceiveRequest(uint8_t *num){
  if(ReadPoint != WritePoint){
    *num = RingBuf[ReadPoint];
    //*num = RingBuf[ReadPoint];
    ReadPoint = (ReadPoint + 1) % RINGSIZE;
  }
  return 0;
}

int Empty_Check(void){
  if(WritePoint == ReadPoint){
    return -1;
  }else{
    return 0;
  }
}

int Read_ENCODER(DD_SSHand_t *dmd){
  int ret;
  ret = DD_I2C2Receive(dmd->add, dmd->data, dmd->data_size);
  if(ret){
    return ret;
  }
  return 0;
}