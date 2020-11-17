/* ===Kisarazu RBKN Library===
 *
 * autor          : Wakabayashi
 * version        : v0.10
 * last update    : 20160831
 *
 * **overview***
 * エンコーダーの値を取得する。
 *
 * TIM3及びTIM4を使用
 */
#include "DD_ENCODER.h"

static int32_t encval1 = 0;
static int32_t encval2 = 0;
static int32_t encval3 = 0;
static int32_t encval4 = 0;

int DD_InitEncoder1(void){
  return MW_EncoderInit(ENCODER1ID);
}

int DD_InitEncoder2(void){
  return MW_EncoderInit(ENCODER2ID);
}

int DD_InitEncoder3(void){
  return MW_EncoderInit(ENCODER3ID);
}

int DD_InitEncoder4(void){
  return MW_EncoderInit(ENCODER4ID);
}

int DD_encoder1update(void){  
  encval1 += (int32_t)MW_GetEncoderVal(ENCODER1ID);
  MW_WriteEncoderVal(ENCODER1ID,0);
    
  return EXIT_SUCCESS;
}

int DD_encoder2update(void){  
  encval2 += (int32_t)MW_GetEncoderVal(ENCODER2ID);
  MW_WriteEncoderVal(ENCODER2ID,0);
    
  return EXIT_SUCCESS;
}

int DD_encoder3update(void){  
  encval3 += (int32_t)MW_GetEncoderVal(ENCODER3ID);
  MW_WriteEncoderVal(ENCODER3ID,0);
    
  return EXIT_SUCCESS;
}

int DD_encoder4update(void){  
  encval4 += (int32_t)MW_GetEncoderVal(ENCODER4ID);
  MW_WriteEncoderVal(ENCODER4ID,0);
    
  return EXIT_SUCCESS;
}

int32_t DD_encoder1Get_int32(void){  
  return encval1;
}

int32_t DD_encoder2Get_int32(void){  
  return encval2;
}

int32_t DD_encoder3Get_int32(void){  
  return encval3;
}

int32_t DD_encoder4Get_int32(void){  
  return encval4;
}

int DD_encoder1reset(void){
  encval1 = 0;
  return EXIT_SUCCESS;
}

int DD_encoder2reset(void){
  encval2 = 0;
  return EXIT_SUCCESS;
}

int DD_encoder3reset(void){
  encval3 = 0;
  return EXIT_SUCCESS;
}

int DD_encoder4reset(void){
  encval4 = 0;
  return EXIT_SUCCESS;
}

int DD_encoderprint(void){
  MW_printf("ENC[%6d][%6d][%6d][%6d]\n",encval1,encval2,encval3,encval4);
  return EXIT_SUCCESS;
}
    
