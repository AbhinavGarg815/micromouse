/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>
#include <limits.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Pair
{
    int x;
    int y;
    struct Pair *next;
} Pair;

typedef struct
{
    Pair *head;
    Pair *tail;
} Queue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define gridSize 6
#define maxPath 256
#define baseSpeed 90
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
int cx1, cy1, cx2, cy2;
int facing = 0;

int centers[4][2];

bool horizontalWalls[gridSize][gridSize];
bool verticalWalls[gridSize][gridSize];

int distance[gridSize][gridSize];

bool visited[gridSize][gridSize];

int directions[4][2];

bool centerFound = false;

int marking[gridSize][gridSize];

int explored[gridSize][gridSize];

uint16_t ir1 = 0;
uint16_t ir2 = 0;
uint16_t ir3 = 0;
uint16_t ir4 = 0;
uint16_t ir5 = 0;
uint16_t ir6 = 0;
uint16_t rawValues[6];

uint16_t leftFrontThreshold = 600;
uint16_t rightFrontThreshold = 600;
uint16_t leftThreshold;
uint16_t rightThreshold;

uint16_t leftIrMin = 1023;
uint16_t leftIrMax = 0;
uint16_t rightIrMin = 1023;
uint16_t rightIrMax = 0;
uint16_t leftFrontIrMin = 1023;
uint16_t leftFrontIrMax = 0;
uint16_t rightFrontIrMin = 1023;
uint16_t rightFrontIrMax = 0;


uint16_t sideIrOffset;
uint16_t diagonalOffset;
uint16_t rightOffset;
uint16_t leftOffset;

float kpIr = 0.025f;
float kdIr = 20.0f;
float kiIr = 0.0f;
float errorKpIr = 0.0f;
float errorKdIr = 0.0f;
float errorKiIr = 0.0f;
float prevErrorIr = 0.0f;
float totalErrorIr = 0.0f;

float kpEncoder = 0.0f;
float kdEncoder = 0.0f;
float errorKpEncoder = 0.0f;
float errorKdEncoder = 0.0f;
float prevErrorEncoder = 0.0f;
float totalErrorEncoder = 0.0f;

int rightMotorSpeed = 0;
int leftMotorSpeed = 0;

const int forwardEncoderCount = 850;
const int turnEncoderCount = 400;

char buffer[60];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t flag = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  flag = 1;
}

void initQueue(Queue *q)
{
    q->head = NULL;
    q->tail = NULL;
}

bool enqueue(Queue *q, Pair *p)
{
    Pair *newPair = malloc(sizeof(Pair));
    if (newPair == NULL)
    {
        return false;
    }
    newPair->x = p->x;
    newPair->y = p->y;
    newPair->next = NULL;

    if (q->tail != NULL)
    {
        q->tail->next = newPair;
    }
    q->tail = newPair;

    if (q->head == NULL)
    {
        q->head = q->tail;
    }
    return true;
}

Pair *dequeue(Queue *q)
{
    if (q->head == NULL)
    {
        return NULL;
    }
    Pair *p = q->head;
    q->head = q->head->next;
    if (q->head == NULL)
    {
        q->tail = NULL;
    }
    return p;
}

void updateIrReading() {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)rawValues, 6);
	while (!flag);

	ir1 = (uint16_t) rawValues[0];
	ir2 = (uint16_t) rawValues[1];
	ir3 = (uint16_t) rawValues[2];
	ir4 = (uint16_t) rawValues[3];
	ir5 = (uint16_t) rawValues[4];
	ir6 = (uint16_t) rawValues[5];

//	snprintf(buffer, sizeof(buffer), "%d	", ir1);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//	snprintf(buffer, sizeof(buffer), "%d	", ir2);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//	snprintf(buffer, sizeof(buffer), "%d	", ir3);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//	snprintf(buffer, sizeof(buffer), "%d	", ir4);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//	snprintf(buffer, sizeof(buffer), "%d	", ir5);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//	snprintf(buffer, sizeof(buffer), "%d\n", ir6);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

	HAL_ADC_Stop(&hadc1);

	flag = 0;
}

uint16_t mapRange(uint16_t value, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void motorSpeed(int rightMotorSpeed, int leftMotorSpeed) {
	 if (rightMotorSpeed > 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	}
	else if (rightMotorSpeed < 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		rightMotorSpeed = -1 * rightMotorSpeed;
	}
	else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	}

	if (leftMotorSpeed > 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	else if (leftMotorSpeed < 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		leftMotorSpeed = -1 * leftMotorSpeed;
	}
	else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}

	rightMotorSpeed = rightMotorSpeed > 150 ? 150 : rightMotorSpeed;
	leftMotorSpeed = leftMotorSpeed > 150 ? 150 : leftMotorSpeed;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, rightMotorSpeed);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, leftMotorSpeed);
}

void encoderPID() {
	uint32_t leftEncoderCount = __HAL_TIM_GET_COUNTER(&htim3);
	uint32_t rightEncoderCount = __HAL_TIM_GET_COUNTER(&htim4);

	errorKpEncoder = (leftEncoderCount - rightEncoderCount);
	prevErrorEncoder = errorKpEncoder;
	errorKdEncoder = errorKpEncoder - prevErrorEncoder;
	totalErrorEncoder = (kpEncoder * errorKpEncoder) + (kdEncoder * errorKdEncoder);

	snprintf(buffer, sizeof(buffer), "%ld	%ld		%f", leftEncoderCount, rightEncoderCount, totalErrorEncoder);
	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

	rightMotorSpeed = baseSpeed + totalErrorEncoder;
	leftMotorSpeed = baseSpeed - totalErrorEncoder;

	rightMotorSpeed = rightMotorSpeed < 50 ? 0 : rightMotorSpeed;
	leftMotorSpeed = leftMotorSpeed < 50 ? 0 : leftMotorSpeed;

	rightMotorSpeed = rightMotorSpeed > 110 ? 110 : rightMotorSpeed;
	leftMotorSpeed = leftMotorSpeed > 110 ? 110 : leftMotorSpeed;

	motorSpeed(rightMotorSpeed, leftMotorSpeed);
}

void irPID() {
	updateIrReading();
//	int x;
	uint16_t left = mapRange(rawValues[5], leftIrMin, leftIrMax, 0, 1023);
	uint16_t right = mapRange(rawValues[0], rightIrMin, rightIrMax, 0, 1023);
//	uint16_t rightFront = mapRange(rawValues[1], rightFrontIrMin, rightFrontIrMax, 0, 1023);
//	uint16_t leftFront = mapRange(rawValues[4], leftFrontIrMin, leftFrontIrMax, 0, 1023);

//	uint16_t left = rawValues[5];
//	uint16_t right = rawValues[0];

	if (rawValues[5] > leftThreshold && rawValues[0] > rightThreshold) {
		errorKpIr = right - left;
//		errorKpIr = (right - left) + (rightFront - leftFront);
//		x = 0;
	}
	else if (rawValues[5] > leftThreshold) {
//		errorKpIr = 2 * (rawValues[3] - rawValues[5] - leftOffset);
		errorKpIr = mapRange(rightThreshold, rightIrMin, rightIrMax, 0, 1023) - left;
//		errorKpIr = (mapRange(rightThreshold, rightIrMin, rightIrMax, 0, 1023) - left) + (rightFront - leftFront);

//		x = 1;
	}
	else if (rawValues[0] > rightThreshold) {
//		errorKpIr = 2 * (rawValues[0] - rawValues[2] - rightOffset);
		errorKpIr = right - mapRange(leftThreshold, leftIrMin, leftIrMax, 0, 1023);
//		errorKpIr = (right - mapRange(leftThreshold, leftIrMin, leftIrMax, 0, 1023)) + (rightFront - leftFront);
//		x = 2;
	}
	else {
		encoderPID();
	}

	errorKdIr = errorKpIr - prevErrorIr;
	errorKiIr = errorKiIr + prevErrorIr;
	totalErrorIr = (kpIr * errorKpIr) + (kdIr * errorKdIr) + (kiIr * errorKiIr);
	prevErrorIr = errorKpIr;

	rightMotorSpeed = baseSpeed + totalErrorIr;
	leftMotorSpeed = baseSpeed - totalErrorIr;

	rightMotorSpeed = rightMotorSpeed < 50 ? 0 : rightMotorSpeed;
	leftMotorSpeed = leftMotorSpeed < 50 ? 0 : leftMotorSpeed;

	rightMotorSpeed = rightMotorSpeed > 110 ? 110 : rightMotorSpeed;
	leftMotorSpeed = leftMotorSpeed > 110 ? 110 : leftMotorSpeed;

	motorSpeed(rightMotorSpeed, leftMotorSpeed);

//	snprintf(buffer, sizeof(buffer), "%d	%f	%d	%d	%d	%d\n", x, totalErrorIr, rightMotorSpeed, leftMotorSpeed, rightMotorSpeed - leftMotorSpeed, rawValues[0] - rawValues[5] - sideIrOffset);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

//	snprintf(buffer, sizeof(buffer), "%d	%f	%d	%d	%d\n", x, totalErrorIr, rawValues[0], rawValues[5],	rawValues[0] - rawValues[5] - sideIrOffset);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

//	snprintf(buffer, sizeof(buffer), "%d	%f	%d	%d	%d\n", x, totalErrorIr, left, right, right - left);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

//	snprintf(buffer, sizeof(buffer), "--%d   %d	  %d    %d--\n", rawValues[0], rawValues[5], rightThreshold, leftThreshold);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

//	snprintf(buffer, sizeof(buffer), "%d	%f	%d	%d	%f\n", x, totalErrorIr, rawValues[0], rawValues[5], errorKpIr);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void turnLeft() {
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);

	uint32_t leftCount = __HAL_TIM_GET_COUNTER(&htim3);
	uint32_t rightCount = __HAL_TIM_GET_COUNTER(&htim4);

	uint32_t targetLeftCount = 65535 - turnEncoderCount;
	uint32_t targetRightCount = turnEncoderCount;

	while ((leftCount > targetLeftCount || leftCount == 0) && (rightCount < targetRightCount)) {
		motorSpeed(70, -70);

		leftCount = __HAL_TIM_GET_COUNTER(&htim3);
		rightCount = __HAL_TIM_GET_COUNTER(&htim4);
//		snprintf(buffer, sizeof(buffer), "LR = %ld %ld %ld %ld\n\r", targetLeftCount, leftCount, targetRightCount, rightCount);
//		HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), 100);
	}

	motorSpeed(-70, 70);
	HAL_Delay(15);

	motorSpeed(0, 0);
}

void turnRight() {
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);

	uint32_t leftCount = __HAL_TIM_GET_COUNTER(&htim3);
	uint32_t rightCount = __HAL_TIM_GET_COUNTER(&htim4);

	uint32_t targetLeftCount = turnEncoderCount;
	uint32_t targetRightCount = 65535 - turnEncoderCount;

	while ((rightCount > targetRightCount || rightCount == 0) && (leftCount < targetLeftCount)) {
		motorSpeed(-70, 70);

		leftCount = __HAL_TIM_GET_COUNTER(&htim3);
		rightCount = __HAL_TIM_GET_COUNTER(&htim4);
//		snprintf(buffer, sizeof(buffer), "LR = %ld %ld %ld %ld\n\r", targetLeftCount, leftCount, targetRightCount, rightCount);
//		HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), 100);
	}

	motorSpeed(70, -70);
	HAL_Delay(15);

	motorSpeed(0, 0);
}

void moveForward() {
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);

	uint32_t cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
	uint32_t cnt2 = __HAL_TIM_GET_COUNTER(&htim4);

	while (cnt1 < forwardEncoderCount && cnt2 < forwardEncoderCount) {
		cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
		cnt2 = __HAL_TIM_GET_COUNTER(&htim4);
//		motorSpeed(50, 52);

		irPID();
	}

	motorSpeed(-60, -60);

	HAL_Delay(35);

	motorSpeed(0, 0);

//	snprintf(buffer, sizeof(buffer), "LR = %ld %ld\n\r", cnt1, cnt2);
//	HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), 100);

	HAL_Delay(2000);
}


bool wallFront() {
	updateIrReading();
	if (rawValues[1] > rightFrontThreshold && rawValues[4] > leftFrontThreshold) {
		return true;
	}
	return false;
}


bool wallRight() {
	updateIrReading();
	if (rawValues[0] > rightThreshold) {
		return true;
	}
	return false;
}


bool wallLeft() {
	updateIrReading();
	if (rawValues[5] > leftThreshold) {
		return true;
	}
	return false;
}


void init()
{
    cx1 = gridSize / 2;
    cy1 = gridSize / 2;
    cx2 = cx1 - 1;
    cy2 = cy1 - 1;

    centers[0][0] = cx1;
    centers[0][1] = cy1;
    centers[1][0] = cx2;
    centers[1][1] = cy1;
    centers[2][0] = cx1;
    centers[2][1] = cy2;
    centers[3][0] = cx2;
    centers[3][1] = cy2;

    memset(visited, false, sizeof(visited));
    memset(marking, 0, sizeof(marking));
    memset(explored, false, sizeof(explored));
}


bool isCellAccessible(int x, int y, int nx, int ny)
{
    int dx = nx - x;
    int dy = ny - y;
    bool isAccessible = true;
    if (dx == 0)
    {
        if (dy == -1)
        {
            isAccessible = !horizontalWalls[nx][ny];
        }
        else
        {
            isAccessible = !horizontalWalls[x][y];
        }
    }
    else
    {
        if (dx == -1)
        {
            isAccessible = !verticalWalls[nx][ny];
        }
        else
        {
            isAccessible = !verticalWalls[x][y];
        }
    }

    if (nx < 0 || nx >= gridSize || ny < 0 || ny >= gridSize)
    {
        isAccessible = false;
    }

    return isAccessible;
}


void updateDirections() {
    if (facing == 0)
    {
        directions[0][0] = 0;
        directions[0][1] = 1;
        directions[1][0] = 1;
        directions[1][1] = 0;
        directions[2][0] = -1;
        directions[2][1] = 0;
        directions[3][0] = 0;
        directions[3][1] = -1;
    }
    else if (facing == 1)
    {
        directions[0][0] = 1;
        directions[0][1] = 0;
        directions[1][0] = 0;
        directions[1][1] = -1;
        directions[2][0] = 0;
        directions[2][1] = 1;
        directions[3][0] = -1;
        directions[3][1] = 0;
    }
    else if (facing == 2)
    {
        directions[0][0] = 0;
        directions[0][1] = -1;
        directions[1][0] = -1;
        directions[1][1] = 0;
        directions[2][0] = 1;
        directions[2][1] = 0;
        directions[3][0] = 0;
        directions[3][1] = 1;
    }
    else
    {
        directions[0][0] = -1;
        directions[0][1] = 0;
        directions[1][0] = 0;
        directions[1][1] = 1;
        directions[2][0] = 0;
        directions[2][1] = -1;
        directions[3][0] = 1;
        directions[3][1] = 0;
    }
}


void floodfill()
{
    for (int i = 0; i < gridSize; i++)
    {
        for (int j = 0; j < gridSize; j++)
        {
            distance[i][j] = 999;
        }
    }

    Queue q;
    initQueue(&q);

    for (int i = 0; i < 4; i++)
    {
        int x = centers[i][0];
        int y = centers[i][1];
        distance[y][x] = 0;
        Pair *p = malloc(sizeof(Pair));
        p->x = x;
        p->y = y;
        enqueue(&q, p);
    }

    while (q.head != NULL)
    {
        Pair *p = dequeue(&q);
        int x = p->x;
        int y = p->y;
        free(p);

        for (int i = 0; i < 4; i++)
        {
            int dx = i == 0 ? 0 : i == 1 ? 1
                              : i == 2   ? 0
                                         : -1;
            int dy = i == 0 ? -1 : i == 1 ? 0
                               : i == 2   ? 1
                                          : 0;
            int nx = x + dx;
            int ny = y + dy;

            if (nx < 0 || nx >= gridSize || ny < 0 || ny >= gridSize)
            {
                continue;
            }

            bool isAccessible = isCellAccessible(x, y, nx, ny);

            if (isAccessible && distance[ny][nx] > distance[y][x] + 1)
            {
                distance[ny][nx] = distance[y][x] + 1;
                Pair *newPair = malloc(sizeof(Pair));
                newPair->x = nx;
                newPair->y = ny;
                enqueue(&q, newPair);
            }
        }
    }
}


void dynamicFloodfill()
{
    for (int i = 0; i < gridSize; i++)
    {
        for (int j = 0; j < gridSize; j++)
        {
            distance[i][j] = 999;
        }
    }

    Queue q;
    initQueue(&q);

    distance[0][0] = 0;
    Pair *p = malloc(sizeof(Pair));
    p->x = 0;
    p->y = 0;
    enqueue(&q, p);

    while (q.head != NULL)
    {
        Pair *p = dequeue(&q);
        int x = p->x;
        int y = p->y;
        free(p);

        for (int i = 0; i < 4; i++)
        {
            int dx = i == 0 ? 0 : i == 1 ? 1
                              : i == 2   ? 0
                                         : -1;
            int dy = i == 0 ? -1 : i == 1 ? 0
                               : i == 2   ? 1
                                          : 0;
            int nx = x + dx;
            int ny = y + dy;

            if (nx < 0 || nx >= gridSize || ny < 0 || ny >= gridSize)
            {
                continue;
            }

            bool isAccessible = isCellAccessible(x, y, nx, ny);

            if (isAccessible && distance[ny][nx] > distance[y][x] + 1)
            {
                distance[ny][nx] = distance[y][x] + 1;
                Pair *newPair = malloc(sizeof(Pair));
                newPair->x = nx;
                newPair->y = ny;
                enqueue(&q, newPair);
            }
        }
    }
}


void exploredFloodfill()
{
    for (int i = 0; i < gridSize; i++)
    {
        for (int j = 0; j < gridSize; j++)
        {
            distance[i][j] = 999;
        }
    }

    Queue q;
    initQueue(&q);

    for (int i = 0; i < 4; i++)
    {
        int x = centers[i][0];
        int y = centers[i][1];
        distance[y][x] = 0;
        Pair *p = malloc(sizeof(Pair));
        p->x = x;
        p->y = y;
        enqueue(&q, p);
    }

    while (q.head != NULL)
    {
        Pair *p = dequeue(&q);
        int x = p->x;
        int y = p->y;
        free(p);

        for (int i = 0; i < 4; i++)
        {
            int dx = i == 0 ? 0 : i == 1 ? 1
                              : i == 2   ? 0
                                         : -1;
            int dy = i == 0 ? -1 : i == 1 ? 0
                               : i == 2   ? 1
                                          : 0;
            int nx = x + dx;
            int ny = y + dy;

            if (nx < 0 || nx >= gridSize || ny < 0 || ny >= gridSize)
            {
                continue;
            }

            bool isAccessible = isCellAccessible(x, y, nx, ny);

            if (isAccessible && distance[ny][nx] > distance[y][x] + 1 && explored[nx][ny] == true)
            {
                distance[ny][nx] = distance[y][x] + 1;
                Pair *newPair = malloc(sizeof(Pair));
                newPair->x = nx;
                newPair->y = ny;
                enqueue(&q, newPair);
            }
        }
    }
}


void mouseControl(int x, int y, int nx, int ny)
{
    int dx = nx - x;
    int dy = ny - y;

    snprintf(buffer, sizeof(buffer), "%d %d %d\n\r", dx, dy, facing);
    HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), 100);

    if (dx == 0 && dy == 1)
    {
        if (facing == 0)
        {
            moveForward();
        }
        else if (facing == 1)
        {
            turnLeft();
            moveForward();
        }
        else if (facing == 2)
        {
            turnLeft();
            turnLeft();
            moveForward();
        }
        else
        {
            turnRight();
            moveForward();
        }
        facing = 0;
    }
    else if (dx == 1 && dy == 0)
    {
        if (facing == 0)
        {
            turnRight();
            moveForward();
        }
        else if (facing == 1)
        {
            moveForward();
        }
        else if (facing == 2)
        {
            turnLeft();
            moveForward();
        }
        else
        {
            turnLeft();
            turnLeft();
            moveForward();
        }
        facing = 1;
    }
    else if (dx == 0 && dy == -1)
    {
        if (facing == 0)
        {
            turnRight();
            turnRight();
            moveForward();
        }
        else if (facing == 1)
        {
            turnRight();
            moveForward();
        }
        else if (facing == 2)
        {
            moveForward();
        }
        else
        {
            turnLeft();
            moveForward();
        }
        facing = 2;
    }
    else
    {
        if (facing == 0)
        {
            turnLeft();
            moveForward();
        }
        else if (facing == 1)
        {
            turnRight();
            turnRight();
            moveForward();
        }
        else if (facing == 2)
        {
            turnRight();
            moveForward();
        }
        else
        {
            moveForward();
        }
        facing = 3;
    }
}

void setWalls(int x, int y)
{
    bool frontWall = wallFront();
    bool rightWall = wallRight();
    bool leftWall = wallLeft();

    snprintf(buffer, sizeof(buffer), "Walls: %d %d   %d %d %d\n\r", x, y, frontWall, rightWall, leftWall);
    HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), 100);

    if (facing == 0)
    {
        if (frontWall)
        {
            horizontalWalls[x][y] = true;
        }
        if (rightWall)
        {
            verticalWalls[x][y] = true;
        }
        if (leftWall)
        {
            if (x > 0)
            {
                verticalWalls[x - 1][y] = true;
            }
        }
    }
    else if (facing == 1)
    {
        if (frontWall)
        {
            verticalWalls[x][y] = true;
        }
        if (rightWall)
        {
            if (y > 0)
            {
                horizontalWalls[x][y - 1] = true;
            }
        }
        if (leftWall)
        {
            horizontalWalls[x][y] = true;
        }
    }
    else if (facing == 2)
    {
        if (frontWall)
        {
            if (y > 0)
            {
                horizontalWalls[x][y - 1] = true;
            }
        }
        if (rightWall)
        {
            if (x > 0)
            {
                verticalWalls[x - 1][y] = true;
            }
        }
        if (leftWall)
        {
            verticalWalls[x][y] = true;
        }
    }
    else
    {
        if (frontWall)
        {
            if (x > 0)
            {
                verticalWalls[x - 1][y] = true;
            }
        }
        if (rightWall)
        {
            horizontalWalls[x][y] = true;
        }
        if (leftWall)
        {
            if (y > 0)
            {
                horizontalWalls[x][y - 1] = true;
            }
        }
    }
}

void exploreCenters(int x, int y) {
    setWalls(x, y);
    if (x == 7 && y == 7)
    {
        mouseControl(x, y, 7, 8);
        setWalls(7, 8);
        mouseControl(7, 8, 8, 8);
        setWalls(8, 8);
        mouseControl(8, 8, 8, 7);
        setWalls(8, 7);
        mouseControl(8, 7, 7, 7);
        setWalls(7, 7);
    }
    else if (x == 8 && y == 7)
    {
        mouseControl(x, y, 8, 8);
        setWalls(8, 8);
        mouseControl(8, 8, 7, 8);
        setWalls(7, 8);
        mouseControl(7, 8, 7, 7);
        setWalls(7, 7);
        mouseControl(7, 7, 8, 7);
    }
    else if (x == 7 && y == 8)
    {
        mouseControl(x, y, 8, 8);
        setWalls(8, 8);
        mouseControl(8, 8, 8, 7);
        setWalls(8, 7);
        mouseControl(8, 7, 7, 7);
        setWalls(7, 7);
        mouseControl(7, 7, 7, 8);
    }
    else
    {
        mouseControl(x, y, 8, 7);
        setWalls(8, 7);
        mouseControl(8, 7, 7, 7);
        setWalls(7, 7);
        mouseControl(7, 7, 7, 8);
        setWalls(7, 8);
        mouseControl(7, 8, 8, 8);
    }

    // floodfill();
    dynamicFloodfill();
}

void dynamicDfs(int x, int y)
{
    setWalls(x, y);

    dynamicFloodfill();

    if (x == 0 && y == 0)
    {
        dynamicFloodfill();
        return;
    }

    visited[x][y] = true;

    updateDirections();

    int i = 0;
    int prevX = x;
    int prevY = y;
    while (i < 4)
    {
        int nx = x + directions[i][0];
        int ny = y + directions[i][1];

        if (nx == prevX && ny == prevY)
        {
            i++;
            continue;
        }

        if (isCellAccessible(x, y, nx, ny) && !visited[nx][ny] && distance[ny][nx] < distance[y][x] && marking[ny][nx] < 2)
        {
            if (explored[nx][ny] == true) {
                i++;
                continue;
            }
            mouseControl(x, y, nx, ny);

            marking[ny][nx]++;
            dynamicDfs(nx, ny);

            mouseControl(nx, ny, x, y);
            i = -1;

            prevX = nx;
            prevY = ny;
        }
        i++;
    }

    explored[x][y] = true;
    visited[x][y] = false;
    return;
}

void dfs(int x, int y)
{
    setWalls(x, y);

    floodfill();

    for (int i = 0; i < 4; i++)
    {
        if (x == centers[i][0] && y == centers[i][1])
        {
            centerFound = true;
            init();
            dynamicFloodfill();
            exploreCenters(x, y);
            dynamicDfs(x, y);
            return;
        }
    }

    visited[x][y] = true;

    updateDirections();

    int i = 0;
    int prevX = x;
    int prevY = y;
    while (i < 4)
    {
        int nx = x + directions[i][0];
        int ny = y + directions[i][1];

        if (nx == prevX && ny == prevY)
        {
            i++;
            continue;
        }

        if (isCellAccessible(x, y, nx, ny) && !visited[nx][ny] && distance[ny][nx] < distance[y][x] && marking[ny][nx] < 2)
        {
            if (explored[nx][ny] == true) {
                i++;
                continue;
            }

            snprintf(buffer, sizeof(buffer), "destination %d %d %d %d\n\r", x, y, nx, ny);
            HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), 100);

            mouseControl(x, y, nx, ny);

            marking[ny][nx]++;
            dfs(nx, ny);

            mouseControl(nx, ny, x, y);
            i = -1;

            prevX = nx;
            prevY = ny;
        }
        i++;
    }

    explored[x][y] = true;
    visited[x][y] = false;
    return;
}

void getShortestPath(int x, int y) {
    updateDirections();

    for (int i = 0; i < 4; i++) {
        if (x == centers[i][0] && y == centers[i][1]) {
            return;
        }
    }

    for (int i = 0; i < 4; i++) {
        int nx = x + directions[i][0];
        int ny = y + directions[i][1];
        if (nx < 0 || nx >= gridSize || ny < 0 || ny >= gridSize) {
            continue;
        }
        if (isCellAccessible(x, y, nx, ny) && distance[ny][nx] < distance[y][x]) {
            mouseControl(x, y, nx, ny);
            getShortestPath(nx, ny);
            break;
        }
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  //STBY

//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);  //Right Motor
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  //Right Motor
//
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //Left Motor
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);	//Left Motor

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //Right Motor
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //Left Motor

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  //Right Encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  //Left Encoder

  HAL_Delay(3000);

  updateIrReading();
  diagonalOffset = rawValues[2] - rawValues[3];
  leftOffset = rawValues[3] - rawValues[5];
  rightOffset = rawValues[0] - rawValues[2];
  rightThreshold = rightIrMin;
  leftThreshold = leftIrMin;



  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

  for (int i = 0; i < 200; i++) {
	  updateIrReading();
	  rightIrMin = rawValues[0] < rightIrMin ? rawValues[0] : rightIrMin;
	  rightIrMax = rawValues[0] > rightIrMax ? rawValues[0] : rightIrMax;
	  leftIrMin = rawValues[5] < leftIrMin ? rawValues[5] : leftIrMin;
	  leftIrMax = rawValues[5] > leftIrMax ? rawValues[5] : leftIrMax;
//	  rightFrontIrMin = rawValues[1] < rightFrontIrMin ? rawValues[1] : rightFrontIrMin;
//	  rightFrontIrMax = rawValues[1] > rightFrontIrMax ? rawValues[1] : rightFrontIrMax;
//	  leftFrontIrMin = rawValues[4] < leftFrontIrMin ? rawValues[4] : leftFrontIrMin;
//	  leftFrontIrMax = rawValues[4] < leftFrontIrMax ? rawValues[4] : leftFrontIrMax;

	  HAL_Delay(30);
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

//  snprintf(buffer, sizeof(buffer), "%d	%d %d %d\n\r", rightIrMin, rightIrMax, leftIrMin, leftIrMax);
//  HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), 100);

  HAL_Delay(3000);

  sideIrOffset = mapRange(rawValues[0], rightIrMin, rightIrMax, 0, 1023) - mapRange(rawValues[5], leftIrMin, leftIrMax, 0, 1023);
//  rightThreshold = rightIrMin;
//  leftThreshold = leftIrMin;
//  rightFrontThreshold = rightFrontIrMin;
//  leftFrontThreshold = leftFrontIrMin;



  init();
  floodfill();
  dfs(0, 0);
  exploredFloodfill();
  getShortestPath(0, 0);



//  char MSG[40];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//	  updateIrReading();
//	  snprintf(buffer, sizeof(buffer), "%d    %d    %d    %d    %d    %d\n\r", rawValues[0], rawValues[1], rawValues[2], rawValues[3], rawValues[4], rawValues[5]);
//	  HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), 100);


//	  int x = 0;
//	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
//		  x = 1;
//		  __HAL_TIM_SET_COUNTER(&htim4, 0);
//		  __HAL_TIM_SET_COUNTER(&htim3, 0);
//	  }

//	  for (int i = 0; i < 255; i = i + 20) {
//		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, i);
//		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, i);
//
//		  uint32_t cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
//		  uint32_t cnt2 = __HAL_TIM_GET_COUNTER(&htim4);
//		  snprintf(buffer, sizeof(buffer), "LR = %ld %ld\n\r", cnt1, cnt2);
//		  HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), 100);
//		  HAL_Delay(50);
//	  }
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//	  HAL_Delay(200 );


//	  irPID();

//	  encoderPID();

//	  moveForward();
//	  HAL_Delay(2000);

//	  turnRight();
//	  HAL_Delay(2000);
//	  snprintf(buffer, sizeof(buffer), "HELLO");
//	  HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), 100);
//	  turnLeft();
//	  HAL_Delay(2000);



//	  init();
//	  floodfill();
//	  dfs(0, 0);
//	  exploredFloodfill();
//	  getShortestPath(0, 0);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
