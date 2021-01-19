#include <LPC17xx.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// type name for pointer to a function
typedef void (*rtosTaskFunc_t)(void *args);

typedef enum {
	READY = 0, 
	RUNNING = 1, 
	BLOCKED = 2,
    INACTIVE = 3,
	TERMINATED = 4
} state_t;

typedef enum {
	LOWEST = 0x00,
	LOW = 0x1,
	MEDIUM = 0x2,
	HIGH = 0x3,
	HIGHEST = 0x4
} priority_t;

typedef struct taskParams {
    // this assumes deadline = period
    uint32_t period;
    uint32_t wcet; // worst-case-execution-time
} taskParams_t;
	
typedef struct TCB {
	uint8_t taskId;
	uint32_t stackPtrAddr;
	state_t state;
	priority_t priority;
    priority_t origPriority;
    struct taskParams params;
    uint32_t prevPeriod;
	struct TCB *next;
	struct TCB *prev;
} TCB_t;

typedef struct queue {
	TCB_t *head;
	TCB_t *tail;
	uint16_t size;
} queue_t;

typedef struct semaphore {
    uint8_t semaId;
    uint8_t maxCount;
    uint8_t count;
} semaphore_t;

typedef struct mutex {
    uint8_t mutexId;
    bool prioInherit;
    uint32_t priorities; // a bit vector to help indicate the highest priority
    uint8_t ownerTaskId;
    uint8_t count;
} mutex_t;
