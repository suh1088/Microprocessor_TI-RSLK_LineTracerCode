#include "msp.h"
#include "clock.h"
#include <stdio.h>
//#include "Tachometer.h"



void systick_init(void){
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = 0x00000005;
}

void systick_wait1us(int s){
    SysTick->LOAD = 48 * s;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000) == 0){}
    //while(SysTick->CTRL & 0x00010000 == 0){}
}

void systick_wait1ms(uint32_t s){
    int i;
    int count = 1000 * s;

    for(i = 0; i < count; i++){
        systick_wait1us(1);
    }
}

void systick_wait1s(int s){
    int i;
    int count = 1000 * s;

    for(i = 0; i < count; i++){
        systick_wait1ms(1);
    }
}

void led_init(){
    // LED Init
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x07;
    P2->OUT &= ~0x07;
}

void IRS_init(){
    //0,2,4,6 IR Emitter
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08;
    P5->OUT &= ~0x08;

    // 1,3,5,7 IR Emitter
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;
    P9->OUT &= ~0x04;

    // 0~7 IR Sensor
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR &= ~0xFF;
}

void IR_usage(int* arr){
    P5->OUT |= 0x08;
    P9->OUT |= 0x04;

    // Make p7.0-P7.7 as output
    P7->DIR = 0xFF;
    // Carges a capacitor
    P7->OUT = 0xFF;
    // Wait for fully charged
    systick_wait1us(10);

    // Make P7.0-P7.7 as input
    P7->DIR = 0x00;
    // Wait for a while
    systick_wait1us(1000);


    int msk = 0x01;
    int i;
    int ab=1;
    for(i = 0; i < 8; i++){
        arr[i] = !!(P7->IN & msk);
        ab *= arr[i];
        msk <<= 1;
    } // masking each sensors

    //printf("%x\n",P7->IN);

    // Turn off IR LEDsa
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;

    systick_wait1us(10);

    return;

}



void pwm_init34(uint16_t period, uint16_t duty3, uint16_t duty4){
    TIMER_A0->CCR[0] = period;

    TIMER_A0->EX0 = 0x0000;

    TIMER_A0->CCTL[3] = 0x0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0x0040;
    TIMER_A0->CCR[4] = duty4;

    TIMER_A0->CTL = 0x02F0;

    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
}

void motor_init(void){
        P3->SEL0 &= ~0xC0;
        P3->SEL1 &= ~0xC0;
        P3->DIR |= 0xC0;
        P3->OUT &= ~0xC0;

        P5->SEL0 &= ~0x30;
        P5->SEL1 &= ~0x30;
        P5->DIR |= 0x30;
        P5->OUT &= ~0x30;

        P2->SEL0 &= ~0xC0;
        P2->SEL1 &= ~0xC0;
        P2->DIR |= 0xC0;
        P2->OUT &= ~0xC0;

        pwm_init34(7500, 0, 0);
}

void move(uint16_t leftDuty, uint16_t rightDuty){
    P3->OUT |= 0xC0;
    TIMER_A0->CCR[3] = leftDuty;
    TIMER_A0->CCR[4] = rightDuty;
}

void left_forward(){
    P5->OUT &= ~0x10;
}

void left_backward(){
    P5->OUT |= 0x10;
}

void right_forward(){
    P5->OUT &= ~0x20;
}

void right_backward(){
    P5->OUT |= 0x20;
}

// 센서 값을 읽어서 0 또는 1로 정규화하고, 활성화된 센서 수를 반환하는 함수
int read_line_sensors(int* sensor_array) {
    // 1. 적외선 발광 (IR Emitter On)
    P5->OUT |= 0x08;
    P9->OUT |= 0x04;

    // 2. 커패시터 충전 (Output High)
    P7->DIR = 0xFF;
    P7->OUT = 0xFF;
    systick_wait1us(10); // 10us 충전

    // 3. 방전 대기 (Input Mode)
    P7->DIR = 0x00;
    systick_wait1us(1000); // 1ms 대기 (검은색은 방전 느림 -> High 유지, 흰색은 방전 빠름 -> Low)

    // 4. 값 읽기
    int active_count = 0;
    int raw_input = P7->IN; // 포트 값을 한 번에 읽음
    int i;
    for(i = 0; i < 8; i++) {
        // 각 비트를 확인하여 1(검은색/라인) 또는 0(흰색/바닥)으로 변환
        if (raw_input & (1 << i)) {
            sensor_array[i] = 1;
            active_count++;
        } else {
            sensor_array[i] = 0;
        }
    }

    // 5. 적외선 끄기
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;
    systick_wait1us(10); // 센서 안정화 대기

    return active_count; // 라인이 감지되었는지 확인용 (0이면 라인 없음)
}





void (*TimerA2Task)(void);
void TimerA2_Init(void(*task)(void), uint16_t period){
    TimerA2Task = task;
    TIMER_A2->CTL = 0x0280;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = (period - 1);
    TIMER_A2->EX0 = 0x0005;
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF00)|0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;
}

void timer_A3_capture_init(){
    P10->SEL0 |= 0x30;
    P10->SEL1 &= ~0x30;
    P10->DIR &= ~0x30;

    TIMER_A3->CTL &= ~0x0030;
    TIMER_A3->CTL = 0x0200;

    TIMER_A3->CCTL[0] = 0x4910;
    TIMER_A3->CCTL[1] = 0x4910;
    TIMER_A3->EX0 &= ~0x0007;

    NVIC->IP[3] = (NVIC->IP[3] & 0x0000FFFF) | 0x40400000;
    NVIC->ISER[0] = 0x0000C000;
    TIMER_A3->CTL |= 0x0024;
}

uint32_t left_count;
uint32_t right_count;

uint16_t period_left;
uint16_t period_right;

uint16_t first_left;
uint16_t first_right;

//void TA2_0_IRQHandler(void){
//    TIMER_A2->CCTL[0] &= ~0x0001;
//    (*TimerA2Task)();
//}

void Tachometer_Get(int32_t *leftSteps, int32_t *rightSteps){
    *leftSteps = left_count;
    *rightSteps = right_count;
}

void TA3_N_IRQHandler(void){
    //printf("TA3_N\n");
    if (TIMER_A3->CCTL[1] & 0x0001) {
        TIMER_A3->CCTL[1] &= ~0x0001;
        left_count++;
    }
}

void TA3_0_IRQHandler(void){
    //printf("TA3_0\n");
    TIMER_A3->CCTL[0] &= ~0X0001;
    right_count++;
}

void Turn90L(){
    left_count = 0;
    right_count = 0;

    right_forward();
    left_backward();
    move(700, 700);
    while(1){
        printf("%d, %d\n", left_count, right_count);
        if(left_count > 30){
            move(0,0);
            printf("stop\n");
            break;
        }
    }

}

void Rot90L() {
    int32_t leftSteps = 0, rightSteps = 0;
    int32_t initialLeft, initialRight;
    #define PUL 180
    // 현재 엔코더 값 저장
    initialLeft = left_count;
    initialRight = right_count;
    //Tachometer_Get(&initialLeft, &initialRight);
    // 왼쪽 바퀴 전진, 오른쪽 바퀴 후진
    right_forward();
    left_backward();
    move(700, 700);
    // 목표 펄스에 도달할 때까지 회전
    while(1) {
        Tachometer_Get(&leftSteps, &rightSteps);

        // 절댓값으로 비교
        int32_t leftDiff = leftSteps - initialLeft;
        int32_t rightDiff = rightSteps - initialRight;

        if(leftDiff < 0) leftDiff = -leftDiff;
        if(rightDiff < 0) rightDiff = -rightDiff;

        // 두 바퀴 모두 목표 펄스 도달 시 정지
        if(leftDiff >= PUL || rightDiff >= PUL) {
            move(0,0);
            break;
        }
    }

    move(0,0);
    systick_wait1ms(100);  // 안정화 시간
    return;
}

void Rot90R() {
    int32_t leftSteps = 0, rightSteps = 0;
    int32_t initialLeft, initialRight;
    #define PUL 180
    // 현재 엔코더 값 저장
    initialLeft = left_count;
    initialRight = right_count;
    //Tachometer_Get(&initialLeft, &initialRight);
    // 왼쪽 바퀴 전진, 오른쪽 바퀴 후진
    left_forward();
    right_backward();
    move(700, 700);
    // 목표 펄스에 도달할 때까지 회전
    while(1) {
        Tachometer_Get(&leftSteps, &rightSteps);

        // 절댓값으로 비교
        int32_t leftDiff = leftSteps - initialLeft;
        int32_t rightDiff = rightSteps - initialRight;

        if(leftDiff < 0) leftDiff = -leftDiff;
        if(rightDiff < 0) rightDiff = -rightDiff;

        // 두 바퀴 모두 목표 펄스 도달 시 정지
        if(leftDiff >= PUL || rightDiff >= PUL) {
            move(0,0);
            break;
        }
    }

    move(0,0);
    systick_wait1ms(100);  // 안정화 시간
    return;
}



void Track_def() {
    // 속도 설정 (배터리 상태에 따라 조절)
    #define HIGH_SPEED 700  // 직진 속도 (너무 빠르면 급커브 이탈)
    #define LOW_SPEED 350
    #define TURN_HIGH  700  // 회전 시 빠른 바퀴
    #define TURN_LOW   400   // 회전 시 느린 바퀴
    #define PIVOT_SPEED 800 // 제자리 회전 속도 (급커브용)

    // 방향 초기화
    left_forward();
    right_forward();

    while(1) {
        // Turn on IR LEDs
        P5->OUT |= 0x08;
        P9->OUT |= 0x04;

        // Make p7.0-P7.7 as output
        P7->DIR = 0xFF;
        // Carges a capacitor
        P7->OUT = 0xFF;
        // Wait for fully charged
        systick_wait1us(10);

        // Make P7.0-P7.7 as input
        P7->DIR = 0x00;
        // Wait for a while
        systick_wait1us(800);

        int arr[8];
        int msk = 0x01;
        int i;
        int active_count = 0;
        int mask = 1;
        int sensorData = P7->IN;

        for(i = 0; i < 8; i++) {
            if(sensorData & (mask)) {
                arr[i] = 1;
                //line_detected = 1;
                active_count++;
            } else {
                arr[i] = 0;
                //all_black = 0;
            }
            mask <<= 1;
        }

        for(i = 0; i < 8; i++){
            printf("%d / ", arr[i]);
        }
        //printf("%d\n", sensorData);
        printf("\n");

        //printf("%x", raw_val);

        // IR OFF
        P5->OUT &= ~0x08;
        P9->OUT &= ~0x04;

        // =====================================================
        // 2. 주행 로직 (우선순위: 정지 > 급회전 > 완만한 회전 > 직진)
        // =====================================================

        // [상황 1] 종료 라인 (All Black) -> 정지
        if (active_count >= 6) {
            move(0, 0);
            P2->OUT &= ~0x07; // LED OFF
            // 필요하다면 여기서 break; 하여 루프 종료
            //break;
        }

        // [상황 2] 직진 (중앙 센서 감지)
        // 가장 흔한 상황이므로 먼저 체크하되, 급커브 센서가 없어야 함
        if (arr[3] && arr[4]) {
            left_forward(); right_forward();
            move(HIGH_SPEED, HIGH_SPEED);
            systick_wait1ms(10);
            P2->OUT &= ~0x07; P2->OUT |= 0x01; // LED Red
        }

//        // [상황 3] 급좌회전 (90도 코너, 왼쪽 끝 센서 감지)
//        // 제자리 회전(Pivot Turn) 사용: 왼쪽 바퀴 후진, 오른쪽 바퀴 전진
//        else if (arr[6] || arr[7]) {
//            right_forward();  // 왼쪽 바퀴 뒤로
//            left_backward();  // 오른쪽 바퀴 앞으로
//            move(PIVOT_SPEED, PIVOT_SPEED);
//            systick_wait1ms(1); // 짧은 딜레이는 OK (관성 제어)
//        }
//
//        // [상황 4] 급우회전 (90도 코너, 오른쪽 끝 센서 감지)
//        else if (arr[0] || arr[1]) {
//            right_backward();   // 왼쪽 바퀴 앞으로
//            left_forward(); // 오른쪽 바퀴 뒤로
//            move(PIVOT_SPEED, PIVOT_SPEED);
//            systick_wait1ms(1);
//        }

        // [상황 5] 완만한 좌회전 (중앙에서 약간 왼쪽으로 벗어남) -> 왼쪽으로 머리 돌리기
        // 수정됨: 왼쪽 센서(4,5)가 닿으면 -> 로봇이 우측으로 치우침 -> 좌회전 필요
        // 좌회전 = 오른쪽 바퀴(R) 빠르게, 왼쪽 바퀴(L) 느리게
        else if (arr[4] || arr[5]) {
            left_backward();
            right_forward();
            move(PIVOT_SPEED, PIVOT_SPEED); // Left=300, Right=700 (수정됨)
            systick_wait1ms(1);
        }

        // [상황 6] 완만한 우회전 (중앙에서 약간 오른쪽으로 벗어남)
        // 우회전 = 왼쪽 바퀴(L) 빠르게, 오른쪽 바퀴(R) 느리게
        else if (arr[2] || arr[3]) {
            left_forward();
            right_backward();
            move(PIVOT_SPEED, PIVOT_SPEED); // Left=700, Right=300 (수정됨)
            systick_wait1ms(1);
        }

        // [상황 7] 라인 없음 (All White)
        // 복잡한 트랙에서는 멈추는 게 안전하지만, 관성 주행을 원하면 이전 상태 유지 필요.
        // 여기서는 안전하게 정지 코드로 둡니다.
        else {
            //move(0, 0);
            P2->OUT &= ~0x07; // LED OFF
        }
    }
}

#define NUM_SENSORS 8
#define BASE_SPEED 700
#define MAX_SPEED 1000
#define MIN_SPEED 0

float Kp = 0.1;
float Ki = 0.0;              // Integral gain
float Kd = 1.5;              // Derivative gain

float previousError = 0;
float integral = 0;
uint8_t lastValidPosition = 3500;  // Center position

uint16_t sensorMin[NUM_SENSORS];
uint16_t sensorMax[NUM_SENSORS];

uint8_t Read_Sensors(void) {
    uint8_t result = 0;

    P5->OUT |= 0x08;
    P9->OUT |= 0x04;
    P7->DIR = 0xFF;
    P7->OUT = 0xFF;
    Clock_Delay1us(10);

    P7->DIR = 0x00;
    systick_wait1us(800);

    result = P7->IN;
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;

    return result;
}

void Track_1() {
    // 곡률 측정 페이즈: 검은 반원 따라 주행하며 데이터 수집
    int arr[8];
    int i;
    int step_count = 0;
    int total_curvature = 0;
    int curve_samples = 0;
    int clockwise = 0;  // 방향 판정 (센서 패턴으로 결정)

    // ===== 페이즈 1: 반원 추적 및 곡률 측정 =====
    while(1) {
        // 센서 읽기
        P5->OUT |= 0x08;
        P9->OUT |= 0x04;
        P7->DIR = 0xFF;
        P7->OUT = 0xFF;
        systick_wait1us(10);
        P7->DIR = 0x00;
        systick_wait1us(800);

        int sensorData = P7->IN;
        int active_count = 0;
        int sensor_position = 0;  // 활성 센서의 가중 위치

        for(i = 0; i < 8; i++) {
            arr[i] = (sensorData & (1 << i)) ? 1 : 0;
            if(arr[i]) {
                active_count++;
                sensor_position += i;
            }
        }

        P5->OUT &= ~0x08;
        P9->OUT &= ~0x04;
        systick_wait1us(10);

        // 방향 판정 (처음 실행시)
        if(step_count == 0 && active_count > 0) {
            if(sensor_position > 20) {
                clockwise = 1;  // 오른쪽 센서 활성 = 반시계 방향
            } else {
                clockwise = 0;  // 왼쪽 센서 활성 = 시계 방향
            }
        }

        // 곡률 측정: 센서 배열에서 곡률값 추정
        // 중앙(3,4)에서 벗어난 정도 = 곡률의 역수
        if(arr[3] && arr[4]) {
            // 직진 구간 (곡률 낮음)
            total_curvature += 1;
        } else if(arr[4] || arr[5]) {
            // 약간 왼쪽으로 구부러짐
            total_curvature += 2;
        } else if(arr[3] || arr[2]) {
            // 약간 오른쪽으로 구부러짐
            total_curvature += 2;
        } else if(arr[6] || arr[7]) {
            // 급하게 왼쪽으로 구부러짐
            total_curvature += 4;
        } else if(arr[1] || arr[0]) {
            // 급하게 오른쪽으로 구부러짐
            total_curvature += 4;
        }
        curve_samples++;

        // 반원 추적 주행 로직 (Track_2 참고)
        left_forward();
        right_forward();

        if(arr[4] && arr[5] && arr[6] && arr[7]) {
            // 급좌회전 필요
            left_backward();
            right_backward();
            move(300, 700);
            systick_wait1ms(80);
        } else if(arr[0] && arr[1] && arr[2] && arr[3]) {
            // 급우회전 필요
            left_backward();
            right_backward();
            move(700, 300);
            systick_wait1ms(80);
        } else if(arr[3] && arr[4]) {
            // 직진
            left_forward();
            right_forward();
            move(650, 650);
            systick_wait1ms(50);
        } else if(arr[6] || arr[5] || arr[4]) {
            // 약간 왼쪽
            left_backward();
            right_backward();
            move(300, 700);
            systick_wait1ms(60);
            left_forward();
            right_forward();
            move(700, 350);
            systick_wait1ms(30);
        } else if(arr[1] || arr[2] || arr[3]) {
            // 약간 오른쪽
            left_backward();
            right_backward();
            move(700, 300);
            systick_wait1ms(60);
            left_forward();
            right_forward();
            move(400, 700);
            systick_wait1ms(30);
        } else {
            // 라인 감지 불가 - 이전 방향 유지
            left_forward();
            right_forward();
            move(600, 600);
            systick_wait1ms(50);
        }

        step_count++;
    }

    // ===== 페이즈 2: 측정된 곡률로 가상 반원 주행 =====
    // 평균 곡률 계산
    int avg_curvature = curve_samples > 0 ? total_curvature / curve_samples : 2;

    // 반원을 따라 이동한 스텝수만큼 동일한 곡률로 주행하며 돌아오기
    int return_steps = step_count;
    int steps_executed = 0;

    // 반대 방향으로 주행 (돌아오기)
    while(steps_executed < return_steps) {
        // 곡률에 따라 모터 속도 조절
        left_forward();
        right_forward();

        if(clockwise) {
            // 시계 방향으로 왔으므로 반시계 방향으로 돌아오기
            if(avg_curvature >= 4) {
                // 급회전
                left_backward();
                right_backward();
                move(700, 300);
                systick_wait1ms(80);
            } else if(avg_curvature >= 2) {
                // 완만한 회전
                left_backward();
                right_backward();
                move(700, 350);
                systick_wait1ms(60);
            } else {
                // 직진
                move(650, 650);
                systick_wait1ms(50);
            }
        } else {
            // 반시계 방향으로 왔으므로 시계 방향으로 돌아오기
            if(avg_curvature >= 4) {
                // 급회전
                left_backward();
                right_backward();
                move(300, 700);
                systick_wait1ms(80);
            } else if(avg_curvature >= 2) {
                // 완만한 회전
                left_backward();
                right_backward();
                move(350, 700);
                systick_wait1ms(60);
            } else {
                // 직진
                move(650, 650);
                systick_wait1ms(50);
            }
        }

        steps_executed++;
    }

    // 종료
    move(0, 0);
    systick_wait1ms(200);
}



void Track_2(){
    while(1){
            // Turn on IR LEDs
            P5->OUT |= 0x08;
            P9->OUT |= 0x04;

            // Make p7.0-P7.7 as output
            P7->DIR = 0xFF;
            // Carges a capacitor
            P7->OUT = 0xFF;
            // Wait for fully charged
            systick_wait1us(10);

            // Make P7.0-P7.7 as input
            P7->DIR = 0x00;
            // Wait for a while
            systick_wait1us(1000);

            int arr[8];
            int msk = 0x01;
            int i;
            int active_count = 0;
            int mask = 1;
            int sensorData = P7->IN;

            for(i = 0; i < 8; i++) {
                if(sensorData & (mask)) {
                    arr[i] = 1;
                    //line_detected = 1;
                    active_count++;
                } else {
                    arr[i] = 0;
                    //all_black = 0;
                }
                mask <<= 1;
            }
            for(i = 0; i < 8; i++){
                printf("%d ", arr[i]);
            }
            printf("\n");

            // Turn off IR LEDsa
            P5->OUT &= ~0x08;
            P9->OUT &= ~0x04;

            systick_wait1us(10);
            // minimize????


            // stop
            if(active_count >= 6){
                left_forward();
                right_forward();
                move(0, 0);
                P2->OUT &= ~0x07; //LED off
                //P2->OUT |= 0x04; //B
                printf("complete!!\n");
                break;
            }

            else if(arr[4] && arr[5] && arr[6] && arr[7]){
                left_forward();
                right_forward();
                move(1200, 1200);
                systick_wait1ms(140);
                move(0,0);
                Rot90L();
                move(0,0);
                systick_wait1us(800);
                P2->OUT &= ~0x07; //LED off
                printf("aL\n");
            }
            else if(arr[0] && arr[1] && arr[2] && arr[3]){
                left_forward();
                right_forward();
                move(1200, 1200);
                systick_wait1ms(140);
                move(0,0);
                Rot90R();
                move(0,0);
                systick_wait1us(800);
                P2->OUT &= ~0x07; //LED off
                printf("aR\n");
            }

            //직진
            else if(arr[3] && arr[4]){
                left_forward();
                right_forward();
                move(1200, 1200);
                systick_wait1ms(50);

                P2->OUT &= ~0x07;
                P2->OUT |= 0x01; //R
                printf("forward\n");
            }

            //약간 왼쪽
            else if(arr[6] || arr[5] || arr[4]){
                left_backward();
                right_forward();
                move(1000, 600);
                systick_wait1ms(100);
                P2->OUT &= ~0x07; //LED off
                printf("sL\n");
            }

            //약간 오른쪽
            else if(arr[1] || arr[2] || arr[3]){

                left_forward();
                right_backward();
                move(600, 1000);
                systick_wait1ms(100);
                P2->OUT &= ~0x07; //LED off
                printf("sR\n");
            }

            //정지
            else{
                left_backward();
                right_backward();
                move(800, 800);
                systick_wait1ms(100);
                P2->OUT &= ~0x07; //LED off
                //P2->OUT |= 0x02; //G
                printf("stop!\n");
                //break;
            }
        }
}


void Track_3(){
    while(1){
        // Turn on IR LEDs
        P5->OUT |= 0x08;
        P9->OUT |= 0x04;

        // Make p7.0-P7.7 as output
        P7->DIR = 0xFF;
        // Carges a capacitor
        P7->OUT = 0xFF;
        // Wait for fully charged
        systick_wait1us(10);

        // Make P7.0-P7.7 as input
        P7->DIR = 0x00;
        // Wait for a while
        systick_wait1us(800);

        int arr[8];
        int msk = 0x01;
        int i;
        int active_count = 0;
        int mask = 1;
        int line_detected = 0;
        int sensorData = P7->IN;

        for(i = 0; i < 8; i++) {
            if(sensorData & (mask)) {
                arr[i] = 1;
                line_detected = 1;
                active_count++;
            } else {
                arr[i] = 0;
                //all_black = 0;
            }
            mask <<= 1;
        }

        for(i = 0; i < 8; i++){
            printf("%d / ", arr[i]);
        }
        //printf("%d\n", sensorData);
        printf("\n");

        // Turn off IR LEDsa
        P5->OUT &= ~0x08;
        P9->OUT &= ~0x04;

        systick_wait1us(10);
        // minimize????


        // stop
        if(active_count >= 6){
            left_forward();
            right_forward();
            move(0, 0);
            P2->OUT &= ~0x07; //LED off
            //P2->OUT |= 0x04; //B
            printf("complete!!\n");
            //break;
        }

        //blind forwarding
        else if(!line_detected){
            left_forward();
            right_forward();
            move(650, 650);
            systick_wait1ms(20);

            P2->OUT &= ~0x07;
            P2->OUT |= 0x01; //R
            printf("blind forwarding\n");
        }

        //직진
        else if(arr[3] || arr[4]){
            left_forward();
            right_forward();
            move(650, 650);
            systick_wait1ms(20);

            P2->OUT &= ~0x07;
            P2->OUT |= 0x01; //R
            printf("forward\n");
        }

        //약간 왼쪽
        else if(arr[6] || arr[5]){
            left_backward();
            right_backward();
            move(350, 700);
            systick_wait1ms(30);
            left_forward();
            right_forward();
            move(700, 400);
            systick_wait1ms(30);
            P2->OUT &= ~0x07; //LED off
            printf("sL\n");
        }

        //약간 오른쪽
        else if(arr[1] || arr[2]){
            left_backward();
            right_backward();
            move(700, 350);
            systick_wait1ms(30);
            left_forward();
            right_forward();
            move(400, 700);
            systick_wait1ms(30);
            P2->OUT &= ~0x07; //LED off
            printf("sR\n");
        }



        else{
            left_forward();
            right_forward();
            move(700, 700);
            systick_wait1ms(20);
            P2->OUT &= ~0x07; //LED off
            //P2->OUT |= 0x02; //G
            printf("stop!\n");
            //break;
        }



    }
}



void Track_4(){
    int status = 0;
    while(1){
        // Turn on IR LEDs
        P5->OUT |= 0x08;
        P9->OUT |= 0x04;

        // Make p7.0-P7.7 as output
        P7->DIR = 0xFF;
        // Carges a capacitor
        P7->OUT = 0xFF;
        // Wait for fully charged
        systick_wait1us(10);

        // Make P7.0-P7.7 as input
        P7->DIR = 0x00;
        // Wait for a while
        systick_wait1us(800);

        int arr[8];
        int msk = 0x01;
        int i;
        int active_count = 0;
        int mask = 1;
        int line_detected = 0;
        int sensorData = P7->IN;

        for(i = 0; i < 8; i++) {
            if(sensorData & (mask)) {
                arr[i] = 1;
                line_detected = 1;
                active_count++;
            } else {
                arr[i] = 0;
                //all_black = 0;
            }
            mask <<= 1;
        }

        for(i = 0; i < 8; i++){
            printf("%d / ", arr[i]);
        }
        //printf("%d\n", sensorData);
        printf("\n");

        // Turn off IR LEDsa
        P5->OUT &= ~0x08;
        P9->OUT &= ~0x04;

        systick_wait1us(10);
        // minimize????

        if(status && active_count>0){
            move(0,0);
        }
        // stop
        if(active_count >= 6){
            left_forward();
            right_forward();
            move(0, 0);
            P2->OUT &= ~0x07; //LED off
            //P2->OUT |= 0x04; //B
            printf("complete!!\n");
            //break;
        }

        //blind forwarding
        else if(!line_detected){
            status = 1;
            left_forward();
            right_forward();
            move(650, 650);
            systick_wait1ms(20);

            P2->OUT &= ~0x07;
            P2->OUT |= 0x01; //R
            printf("blind forwarding\n");
        }

        //직진
        else if(arr[3] && arr[4]){
            left_forward();
            right_forward();
            move(650, 650);
            systick_wait1ms(20);

            P2->OUT &= ~0x07;
            P2->OUT |= 0x01; //R
            printf("forward\n");
        }

        //약간 왼쪽
        else if(arr[6] || arr[5] || arr[4]){
            left_backward();
            right_backward();
            move(300, 700);
            systick_wait1ms(80);
            left_forward();
            right_forward();
            move(700, 350);
            systick_wait1ms(40);
            P2->OUT &= ~0x07; //LED off
            printf("sL\n");
        }

        //약간 오른쪽
        else if(arr[1] || arr[2] || arr[3]){
            left_backward();
            right_backward();
            move(700, 300);
            systick_wait1ms(80);
            left_forward();
            right_forward();
            move(400, 700);
            systick_wait1ms(40);
            P2->OUT &= ~0x07; //LED off
            printf("sR\n");
        }



        else{
            left_forward();
            right_forward();
            move(700, 700);
            systick_wait1ms(20);
            P2->OUT &= ~0x07; //LED off
            //P2->OUT |= 0x02; //G
            printf("stop!\n");
            //break;
        }



    }
}



void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
    Clock_Init48MHz();
    led_init();
    systick_init();
    IRS_init();
    motor_init();


    //Tachometer_Init();

    systick_wait1s(1);


    //LED_change();
//  IR_usage();


    timer_A3_capture_init();
    //TimerA2_Init(&goTurn, 50000);


    //goTurn();

    //Turn90L();

    //C, P, G
    //Track_1_C();
    Track_2();
    //Track_3();
    //Track_4();
    //Track_def(); // test this!

    while(1){

    }
}




