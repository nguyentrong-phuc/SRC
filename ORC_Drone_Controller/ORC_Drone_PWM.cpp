/*------------------------------------------------------------
  ORC_Drone_PWM.h - Library for manipulting with PWM data.
  Created by Dang Xuan Ba, October 07, 2024.
  Updated by Dang Xuan Ba, October 07, 2024.
  Released into the public domain.
  PWM_min = 0; PWM_max = 1800
-------------------------------------------------------------*/
#include "Arduino.h"
#include "ORC_Drone_PWM.h"
#include "ORCType.h"
/*----------------------------------------
Initialization
------------------------------------------*/
ORC_Drone_PWM::ORC_Drone_PWM()
{
  _PWM_min = 100; //Set PWM_min value (default: 100);
  _PWM_max = 1800; //Set PWM_max value (default: 1800);
}
/*----------------------------------------
BEGIN OF PWM FUNCTIONS
------------------------------------------*/
// #if defined(ARDUINO_ARCH_STM32)
#if defined(STM32F1)
  /*----PWM SETUP-----------------*/
  void ORC_Drone_PWM::PWMSetup()
  {
    this->STMPwmTimer3Setup();
  }
  /*----------------------------------------------------------
   * CAI DAT TIMER 3 CHO PWM: CHANNEL 1->4
  CH1->PA6     CH2->PA7     CH3->PB0      CH4->PB1
  -----------------------------------------------------------*/
  void ORC_Drone_PWM::STMPwmTimer3Setup()
  {
    // Kích hoạt xung nhịp cho TIM3
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    // Kích hoạt clock cho GPIOA, GPIOB và Timer 3
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    // Cấu hình prescaler và period cho Timer 3
    TIM3->PSC = 36-1;  // Prescale = 36 -> tần số Timer_3 = 2MHz
    TIM3->ARR = 5000;  // Auto-reload register = 5000 -> tần số PWM = 400Hz
    // Cấu hình PA6 và PA7 ở chế độ Alternate Function Push-Pull
    GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOA->CRL |= (GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1);
    // Cấu hình PB0 và PB1 ở chế độ Alternate Function Push-Pull
    GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOB->CRL |= (GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1);

    /* CẤU HÌNH CÁC KÊNH TIMER 3 */
    /* KÊNH 1 */
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M; // Xóa các bit OC1M
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // Chế độ PWM1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Kích hoạt preload register cho kênh 
    // Kích hoạt kênh 1 và đặt nó ở trạng thái không đảo
    TIM3->CCER |= TIM_CCER_CC1E; // Kích hoạt kênh 1
    TIM3->CCER &= ~TIM_CCER_CC1P;
    //* KÊNH 2 */
    TIM3->CCMR1 &= ~TIM_CCMR1_OC2M; // Xóa các bit OC2M
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // Chế độ PWM1
    TIM3->CCMR1 |= TIM_CCMR1_OC2PE; // Kích hoạt preload register cho kênh 
    // Kích hoạt kênh 2 và đặt nó ở trạng thái không đảo
    TIM3->CCER |= TIM_CCER_CC2E; // Kích hoạt kênh 2
    TIM3->CCER &= ~TIM_CCER_CC2P;
    /* KÊNH 3 */
    TIM3->CCMR2 &= ~TIM_CCMR2_OC3M; // Xóa các bit OC3M
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // Chế độ PWM1
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE;   // Kích hoạt preload register cho kênh 
    // Kích hoạt kênh 3 và đặt nó ở trạng thái không đảo
    TIM3->CCER |= TIM_CCER_CC3E; // Kích hoạt kênh 3
    TIM3->CCER &= ~TIM_CCER_CC3P;
    /* KÊNH 4 */
    TIM3->CCMR2 &= ~TIM_CCMR2_OC4M;    // Xóa các bit OC4M
    TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // Chế độ PWM1
    TIM3->CCMR2 |= TIM_CCMR2_OC4PE;   // Kích hoạt preload register cho kênh 4
    // Kích hoạt kênh 4 và đặt nó ở trạng thái không đảo
    TIM3->CCER |= TIM_CCER_CC4E;
    TIM3->CCER &= ~TIM_CCER_CC4P;

    // Kích hoạt bộ đếm và cho phép tự động tải lại
    TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
  }
  /*----------------------------------------------------------
   * Dat gia tri cho cac chan
  -----------------------------------------------------------*/
  void ORC_Drone_PWM::setPWM(unsigned int Pwm1, unsigned int Pwm2, unsigned int Pwm3, unsigned int Pwm4)
  {
    unsigned int temp1 = 0;
    //Arm 1
    temp1 = (Pwm1 > (this->_PWM_max))?(this->_PWM_max):(Pwm1 < (this->_PWM_min)?(this->_PWM_min):Pwm1);
    TIM3->CCR1 = (temp1 + 2000); // CH1->PA6
    //Arm 2
    temp1 = (Pwm2 > (this->_PWM_max))?(this->_PWM_max):(Pwm2 < (this->_PWM_min)?(this->_PWM_min):Pwm2);  
    TIM3->CCR2 = (temp1 + 2000); // CH2->PA7
    //Arm 3
    temp1 = (Pwm3 > (this->_PWM_max))?(this->_PWM_max):(Pwm3 < (this->_PWM_min)?(this->_PWM_min):Pwm3);  
    TIM3->CCR3 = (temp1 + 2000); // CH3->PB0
    //Arm 4
    temp1 = (Pwm4 > (this->_PWM_max))?(this->_PWM_max):(Pwm4 < (this->_PWM_min)?(this->_PWM_min):Pwm4);
    TIM3->CCR4 = (temp1 + 2000); // CH4->PB1
  }

#else
  /*----PWM SETUP-----------------*/
  void ORC_Drone_PWM::PWMSetup()
  {
    this->PwmTimer1Setup();
    this->PwmTimer4Setup();
  }
  /*------------------------------------------------------------
  CAI DAT TIMMER 1 CHO PWM: OC1A -> D11; OC1B -> D12
  --------------------------------------------------------------*/
  void ORC_Drone_PWM::PwmTimer1Setup()
  {
    #ifdef ESP32
    bool test = true;
    #elif defined(ARDUINO_ARCH_SAM)
    bool test = true;
    #else  
      TCCR1A = 0;
      TCCR1B = 0;
      // RESET lại 2 thanh ghi
      DDRB |= (1 << PB6);// PB6 - D12
      DDRB |= (1 << PB5);// PB5 - D11
      
      TCCR1A |= (1 << WGM11);
      TCCR1B |= (1 << WGM12) | (1 << WGM13);
      // chọn Fast PWM, chế độ chọn TOP_value tự do  ICR1
      TCCR1A |= (1 << COM1A1);// đầu ra kiểu thường cho PD5 (none-inverting)
      TCCR1A |= (1 << COM1B1);// đầu ra kiểu thường cho PD6 (none-inverting)
      // So sánh thường( none-inverting)
      
      TCCR1B |= (1 << CS11); //prescaler = 8
      // 16MHz/8/250Hz = 8000(4ms)
      // 1ms - 2ms = 2000 - 4000
      OCR1A = 2000;
      OCR1B = 2000;
      ICR1 = 5000;
    #endif
  }
  /*----------  ------------------------------------------------
   * CAI DAT TIMMER 4 CHO PWM: OC4A -> D6; OC4B -> D7
  -----------------------------------------------------------*/
  void ORC_Drone_PWM::PwmTimer4Setup()
  {
    #ifdef ESP32
    bool test = true;
    #elif defined(ARDUINO_ARCH_SAM)
    bool test = true;
    #else 
      TCCR4A = 0;
      TCCR4B = 0;
      // RESET lại 2 thanh ghi
      DDRH |= (1 << PB4);// PH4 - D7
      DDRH |= (1 << PB3);// PH3 - D6
      
      TCCR4A |= (1 << WGM41);
      TCCR4B |= (1 << WGM42) | (1 << WGM13);
      // chọn Fast PWM, chế độ chọn TOP_value tự do  ICR1
      TCCR4A |= (1 << COM4A1);// đầu ra kiểu thường cho PD5 (none-inverting)
      TCCR4A |= (1 << COM4B1);// đầu ra kiểu thường cho PD6 (none-inverting)
      // So sánh thường( none-inverting)
      
      TCCR4B |= (1 << CS41); //prescaler = 1
      // 16MHz/8/250Hz = 8000(4ms)
      // 1ms - 2ms = 2000 - 4000
      OCR4A = 2000;
      OCR4B = 2000;
      ICR4 = 5000;
    #endif
  }
  /*----------------------------------------------------------
   * Dat gia tri cho cac chan
  -----------------------------------------------------------*/
  void ORC_Drone_PWM::setPWM(unsigned int Pwm1, unsigned int Pwm2, unsigned int Pwm3, unsigned int Pwm4)
  {                     
    unsigned int temp1 = 0;
    #ifdef ESP32
      bool test = true;
    #elif defined(ARDUINO_ARCH_SAM)
      bool test = true;
    #else 
      //Arm 1
      temp1 = (Pwm1 > (this->_PWM_max))?(this->_PWM_max):(Pwm1 < (this->_PWM_min)?(this->_PWM_min):Pwm1);
      OCR1A = (temp1+2000);// D11
      //Arm 2
      temp1 = (Pwm2 > (this->_PWM_max))?(this->_PWM_max):(Pwm2 < (this->_PWM_min)?(this->_PWM_min):Pwm2);  
      OCR1B = (temp1+2000);// D12
      //Arm 3
      temp1 = (Pwm3 > (this->_PWM_max))?(this->_PWM_max):(Pwm3 < (this->_PWM_min)?(this->_PWM_min):Pwm3);  
      OCR4A = (temp1+2000);// D6
      //Arm 4
      temp1 = (Pwm4 > (this->_PWM_max))?(this->_PWM_max):(Pwm4 < (this->_PWM_min)?(this->_PWM_min):Pwm4);
      OCR4B = (temp1+2000);// D7
    #endif
  }       
#endif

/*----------------------------------------------------------
 * Dat gia tri PWM
-----------------------------------------------------------*/
void ORC_Drone_PWM::setPWMFromDrone(orcdrone_data *drone_in, uint16_t activeIn)
{
  if(drone_in->desiredData.EmerCommand){
    setPWM(0, 0, 0, 0);
  }else
	if (fabs(drone_in->ebimu_data.roll) > FAIL_ANGLE|| fabs(drone_in->ebimu_data.pitch) > FAIL_ANGLE){ //Truong hop an toan
		drone_in->desiredData.EmerCommand = true;
		setPWM(0, 0, 0, 0);
		while(1)
		 delay(10);
	}else{
		if(drone_in->FlyingEnable < FlyingEnableThres){
			setPWM(PWM_TEST_LEVEL*activeIn, PWM_TEST_LEVEL*activeIn, PWM_TEST_LEVEL*activeIn, PWM_TEST_LEVEL*activeIn);
		}else		
			setPWM(drone_in->Motor1_Speed*activeIn, drone_in->Motor2_Speed*activeIn, drone_in->Motor3_Speed*activeIn, drone_in->Motor4_Speed*activeIn);
	}
}
/*----------------------------------------------------------
 * Dat gia tri PWM_min, PWM_max
-----------------------------------------------------------*/
void ORC_Drone_PWM::setPWMMaxMin(unsigned int PWM_min, unsigned int PWM_max)
{
  _PWM_min = (PWM_min < PWM_max)?PWM_min:PWM_max;
  _PWM_max = (PWM_max > PWM_min)?PWM_max:PWM_min;
}
//----------------------END OF PWM FUNCTIONS--------------------------------------------------------
