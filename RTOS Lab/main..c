#include <LPC17xx.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "types.h"

/*
Testing flags:
		- TEST_CONTEXT_SWITCH
				> tests context switching
					all tasks have the same priority
		-	TEST_PRIORITY_QUEUES
				> tests the use of the priority queues
		- TEST_FPP
				> test FPP scheduling
		- TEST_SEMAPHORES
				> tests the blocking semaphores
		- TEST_MUTEXES
				> tests the robust mutexes with priority inheritence
*/

#define TEST_MUTEXES

#define PERIOD 100
#define STACK_SIZE 1024
#define MAIN_STACK_SIZE 2048
#define NUM_TASKS 6
#define NUM_PRIORITIES 5
#define NUM_SEMAPHORES 6
#define NUM_MUTEXES 6

// Global Vars
uint32_t msTicks = 0;
uint8_t numTasks = 0; // keeps track of number of created tasks
TCB_t TASKS[NUM_TASKS];
TCB_t *runningTask, *nextTask;
uint32_t runningTaskStackAddr, nextTaskStackAddr;
taskParams_t taskParameters[NUM_TASKS];
queue_t PRIORITY_QUEUES[NUM_PRIORITIES]; 
uint8_t queues = 0b00000000;
uint8_t numSemas = 0;
semaphore_t SEMAPHORES[NUM_SEMAPHORES];
uint8_t numMuts = 0;
mutex_t MUTEXES[NUM_MUTEXES];
uint32_t fppCount[NUM_TASKS]; //to help simulate longer tasks and keep track of how long each need to run for

void initQueue (queue_t* q) {
	q -> head = NULL;
	q -> tail = NULL;
	q -> size = 0;
}

bool osKernelInitialize(void) {
	uint32_t *vectorTable = 0x0;
	uint32_t mainStack = vectorTable[0];

	uint32_t addr = mainStack - MAIN_STACK_SIZE;
	for (int i=5; i>=0; i--) {
		TASKS[i].stackPtrAddr = addr;
		TASKS[i].taskId = i;
		TASKS[i].state = TERMINATED;
		TASKS[i].priority = LOWEST;
		TASKS[i].next = NULL;
		TASKS[i].prev = NULL;
		
		addr -= STACK_SIZE;
	}
	// initialize the priority queues
	for (int i = 0; i < NUM_PRIORITIES; i++) {
		initQueue(&PRIORITY_QUEUES[i]);
	}
	return true;
}

queue_t* findPrioQueue(priority_t priority) {
	// samePrio = null -> find the highest, non-empty priority queue
	// else find next ready task with same priority
	if (priority) {
		return &PRIORITY_QUEUES[priority];
	} else {
		return &PRIORITY_QUEUES[31 - __clz(queues)];
	}
}

void updateQueuesBitVector(priority_t prio) {
	// update the queues bit vector if 
	if (PRIORITY_QUEUES[prio].size == 0) {
		queues &= ~(1 << prio);
	} else {
		queues |= (1 << prio);
	}
}

void enqueue(queue_t *q, TCB_t *task) {
	// enqueues tasks to given priority queue
	if (q -> size != 0) {
		task -> prev = q -> tail;
		q -> tail -> next = task;
	} else {
		// empty queue
		task -> prev = NULL;
		q -> head = task;	
		task -> next = NULL;
	}
	q -> tail = task;
	q -> size ++;
	//queues |= (1 << (task -> priority));
	updateQueuesBitVector(task -> priority);
}

TCB_t* dequeue(queue_t *q) {
	// deqeues next task from its priority_t queue
	TCB_t *nextReadyTask = q -> head;
	if (nextReadyTask -> taskId == 0) {
		// this is the idle task, so it shouldn't be dequeued
		return nextReadyTask;
	}
	
	q -> head = nextReadyTask -> next;
	q -> head -> prev = NULL;
	nextReadyTask -> next = NULL;
	q -> size --;
	if (q -> size == 0) 
		q -> tail = NULL;
	updateQueuesBitVector(nextReadyTask -> priority);
	return nextReadyTask;
}

semaphore_t* osSemaphoreNew(uint8_t maxCount, uint8_t initialVal, void* arg) {
	SEMAPHORES[numSemas].maxCount = maxCount;
	SEMAPHORES[numSemas].count = initialVal;
	SEMAPHORES[numSemas].semaId = numSemas;
	numSemas ++;
	return &SEMAPHORES[numSemas - 1];
}

void osSemaphoreAcquire(semaphore_t* sema) {
	// wait on the semaphore. try to decrement counter. if 0, then block
	uint8_t taskId = runningTask -> taskId;
	while(1) {
		if (sema -> count == 0) {
			TASKS[taskId].state = BLOCKED;
		} else {
			TASKS[taskId].state = RUNNING;
			sema -> count --;
			break;
		}
	}
}

void osSemaphoreRelease(semaphore_t* sema) {
	// signal the semaphore. increment the counter
	sema -> count ++;
}

mutex_t* osMutexNew(bool prioInherit) {
	// for now, mutexes will all be robust and non-recurrsive
	MUTEXES[numMuts].mutexId = numMuts;
	MUTEXES[numMuts].prioInherit = prioInherit;
	MUTEXES[numMuts].count = 1;
	numMuts ++;
	return &MUTEXES[numMuts - 1];
}

uint8_t findMutexPrio(mutex_t* mut) {
	uint32_t check = __clz(mut -> priorities);
	return 31 - __clz(mut -> priorities);
}

void updateMutexBitVector(mutex_t* mut, priority_t prio, bool acquired) {
	// update the queues bit vector if 
	if (acquired) {
		mut -> priorities |= (1 << prio);
	} else {
		mut -> priorities &= ~(1 << prio);
	}
}

void taskPrioInherit(mutex_t *mut, TCB_t *task) {
	// task currently holding the mutex has its priority temporarily set to that of the highest priority task being blocked 
	uint8_t prio = findMutexPrio(mut);
	if (prio > task -> priority) {
		task -> origPriority = task -> priority;
		task -> priority = prio;
	}
}

void taskPrioReset(mutex_t *mut) {
	TASKS[mut -> ownerTaskId].priority = TASKS[mut -> ownerTaskId].origPriority;
}

void osMutexAcquire(mutex_t *mut) {
	uint8_t taskId = runningTask -> taskId;
	if (mut -> count == 0) {
		TASKS[taskId].state = BLOCKED;
		enqueue(&PRIORITY_QUEUES[TASKS[taskId].priority], &TASKS[taskId]);
	}
	while(1) {
		if (mut -> count > 0) {
			updateMutexBitVector(mut, TASKS[taskId].priority, true);
			mut -> ownerTaskId = taskId;
			mut -> count --;
			TASKS[taskId].state = RUNNING;
			if (mut -> prioInherit) {
				taskPrioInherit(mut, &TASKS[taskId]);
			}			
			break;
		}
	}
}

void osMutexRelease(mutex_t* mut) {
	if (mut -> ownerTaskId == runningTask -> taskId) {
		taskPrioReset(mut);
		mut -> count ++;
		updateMutexBitVector(mut, TASKS[mut -> ownerTaskId].priority, false);
		mut -> ownerTaskId = NULL;
	}
}

void releaseTasks() {
	uint32_t currTicks = msTicks;
	for (int i = 1; i < numTasks; i++) {
		// start at 1 because TASKS[0] is the idle task
		if (currTicks / TASKS[i].params.period > TASKS[i].prevPeriod) {
			TASKS[i].prevPeriod = currTicks / TASKS[i].params.period;
			if (TASKS[i].state == INACTIVE ) {
				TASKS[i].state = READY;
				enqueue(&PRIORITY_QUEUES[TASKS[i].priority], &TASKS[i]);
			}
		}
	}
}

void SysTick_Handler(void) {
  msTicks++;

	#ifndef TEST_CONTEXT_SWITCH
	releaseTasks();
	uint32_t check = 31 - __clz(queues);
	if (runningTask -> state == INACTIVE || runningTask -> state == TERMINATED) {
		// task just finished its execution or got terminated
		nextTask = dequeue(findPrioQueue(NULL));
		nextTaskStackAddr = nextTask -> stackPtrAddr;
		runningTaskStackAddr = runningTask -> stackPtrAddr;
		
		// set PENDSVSET bit to 1 to change PendSV exception state to pending
		SCB -> ICSR |= (1 << 28);
	}	else if ((31 - __clz(queues)) >= runningTask -> priority ) {
		// no context switch required if there is no ready task of same or higher priority
		nextTask = dequeue(findPrioQueue(NULL));
		nextTaskStackAddr = nextTask -> stackPtrAddr;
		runningTaskStackAddr = runningTask -> stackPtrAddr;
		
		if (runningTask -> state == RUNNING) {
			enqueue(&PRIORITY_QUEUES[runningTask -> priority], runningTask);
		}
		// set PENDSVSET bit to 1 to change PendSV exception state to pending
		SCB -> ICSR |= (1 << 28);
	}
	#endif
	
	#ifdef TEST_CONTEXT_SWITCH
	if (msTicks % 3 == 0)
	{
		if (runningTask -> taskId == 0)
		{
			runningTask -> taskId = 2;
		}
		else if (runningTask -> taskId == 2)
		{
			runningTask -> taskId = 1;
		}
		else if (runningTask -> taskId == 1)
		{
			runningTask -> taskId = 0;
		}
		nextTask = &TASKS[runningTask -> taskId];
		nextTaskStackAddr = nextTask -> stackPtrAddr;
		
		SCB -> ICSR |= 1 << 28;
	}
	#endif
	
}

__asm void contextSwitch(void) {
	// move the process stack pointer into R0 and main stack pointer to R1
	// main stack used for interupt handlers while process stack used for thread execution
	MRS 	R1, MSP
	MRS		R0, PSP

	// push register contenets onto the current task's TCB
	STMFD R0!, {R4-R11}

	// record the current stack pointer address in the current task's TCB
	LDR		R4, =__cpp(&runningTaskStackAddr)
	STR 	R0, [R4]
	// load the next task's top of stack address ino the stack pointer
	LDR		R4,=__cpp(&nextTaskStackAddr)
	LDR		R0, [R4]
	
	// pop register contents from the next task's stack
	LDMFD R0!, {R4-R11}

	MSR 	PSP, R0
	MSR 	MSP, R1

  // return from handler
	BX		LR
}


void PendSV_Handler(void) {	
	contextSwitch();
	TASKS[runningTask -> taskId].stackPtrAddr = runningTaskStackAddr;
	if (TASKS[runningTask -> taskId].state == RUNNING) {
		TASKS[runningTask -> taskId].state = READY;
	}

	TASKS[nextTask -> taskId].state = RUNNING;
	
	runningTaskStackAddr = TASKS[nextTask -> taskId].stackPtrAddr;
	runningTask = nextTask;
}

void osThreadSetPriority(uint8_t id, priority_t prio) {
	TASKS[id].priority = prio;
}


void osThreadExit() {
	uint8_t id = runningTask -> taskId;
	TASKS[id].state = TERMINATED;
	// need to delete contents of its stack?
	numTasks --;
}

void osThreadYield() {
	// yields control to next ready task of same priority
	queue_t *q = findPrioQueue(runningTask -> priority);
	if (q -> size > 0) {
		nextTask = dequeue(q);
		nextTaskStackAddr = nextTask -> stackPtrAddr;
		runningTaskStackAddr = runningTask -> stackPtrAddr;
		
		enqueue(&PRIORITY_QUEUES[runningTask -> priority], runningTask);
		SCB -> ICSR |= (1 << 28);
	}
}

void osEventFlagsWait() {
	//TODO
}

void osDelay(uint8_t ms) {
	// delay in ms
	uint32_t start = msTicks;
	while (msTicks < start + ms);
}

void t0(void* arg) {
	// this is the idle function
	while(1) {
		__disable_irq();
		printf("\nIdle");
		__enable_irq();

	}
}

void t1(void* arg) {
	while(1) {
		if(TASKS[1].state != TERMINATED) {
			#if defined TEST_CONTEXT_SWITCH || defined TEST_PRIORITY_QUEUES
			__disable_irq();
			printf("\nTask 1");
			__enable_irq();
			osThreadYield();
			#endif
			
			#ifdef TEST_FPP
				if (fppCount[1] > 0) {
					__disable_irq();
					printf("\nt1 -> %d. ms: %d. prev: %d", fppCount[1], msTicks, TASKS[1].prevPeriod);
					__enable_irq();
					fppCount[1] --;
				} else {
					TASKS[1].state = INACTIVE;
					fppCount[1] = 4;
				}
			#endif
				
			#ifdef TEST_MUTEXES
			__disable_irq();
			printf("\nt1");
			__enable_irq();
			osThreadYield();
			#endif
		}
	}
}

void t2(void* arg) {
	while(1) {
		if(TASKS[2].state != TERMINATED) {
			#if defined TEST_CONTEXT_SWITCH || defined TEST_PRIORITY_QUEUES
			__disable_irq();
			printf("\nTask 2");
			__enable_irq();
			osThreadYield();
			#endif
			
			#ifdef TEST_FPP
				if (fppCount[2] > 0) {
					__disable_irq();
					printf("\nt2 -> %d. ms: %d. prev: %d", fppCount[2], msTicks, TASKS[2].prevPeriod);
					__enable_irq();
					fppCount[2] --;
				} else {
					TASKS[2].state = INACTIVE;
					fppCount[2] = 2;
				}
			#endif
				
			#ifdef TEST_MUTEXES
			__disable_irq();
			printf("\nt2");
			__enable_irq();
			osThreadYield();
			#endif
		}
	}
}

void t3(void* arg) {
	while(1) {
		if(TASKS[3].state != TERMINATED) {
			#if defined TEST_CONTEXT_SWITCH || defined TEST_PRIORITY_QUEUES	
			__disable_irq();
			printf("\nTask 3");
			__enable_irq();
			osThreadYield();
			#endif

			#ifdef TEST_SEMAPHORES		
			osSemaphoreAcquire(&SEMAPHORES[0]);
			osDelay(10);
			__disable_irq();
			printf("\nsem-3");
			__enable_irq();
			osSemaphoreRelease(&SEMAPHORES[0]);
			osThreadYield();
			#endif
			
			#ifdef TEST_FPP
				if (fppCount[3] > 0) {
					__disable_irq();
					printf("\nt3 -> %d. ms: %d. prev: %d", fppCount[3], msTicks, TASKS[3].prevPeriod);
					__enable_irq();
					fppCount[3] --;
				} else {
					TASKS[3].state = INACTIVE;
					fppCount[3] = 1;
				}
			#endif
			
			#ifdef TEST_MUTEXES
			osDelay(2);
			osMutexAcquire(&MUTEXES[0]);
			__disable_irq();
			printf("\nt3 - mut");
			__enable_irq();
			osMutexRelease(&MUTEXES[0]);
			#endif
		}
	}
}

void t4(void* arg) {
	while(1) {
		if (TASKS[4].state != TERMINATED) {
			#ifdef TEST_SEMAPHORES
			osSemaphoreAcquire(&SEMAPHORES[0]);
			__disable_irq();
			printf("\nsem-4");
			__enable_irq();
			osThreadYield();
			osSemaphoreRelease(&SEMAPHORES[0]);
			#endif
			
			#ifdef TEST_MUTEXES
			osMutexAcquire(&MUTEXES[0]);
			__disable_irq();
			printf("\nt4 - mut");
			__enable_irq();
			osMutexRelease(&MUTEXES[0]);
			#endif
		}
	}
}

void t5(void* arg) {
	while(1) {
		if (TASKS[5].state != TERMINATED) {
			#ifdef TEST_SEMAPHORES
			osSemaphoreAcquire(&SEMAPHORES[0]);
			__disable_irq();
			printf("\nsem-5");
			__enable_irq();
			osSemaphoreRelease(&SEMAPHORES[0]);
			osThreadYield();
			#endif
			
			#ifdef TEST_MUTEXES
			osMutexAcquire(&MUTEXES[0]);
			__disable_irq();
			printf("\nt5 - mut");
			__enable_irq();
			osMutexRelease(&MUTEXES[0]);
			osDelay(5);
			#endif
		}
	}
}

void osThreadNew(rtosTaskFunc_t task, void *arg, priority_t prio) {
	TCB_t *newTask = &TASKS[numTasks];
	if (arg) {
		taskParams_t *params = (taskParams_t*) arg;
		newTask -> params = *params;
	}
	newTask -> priority = prio;
	newTask -> origPriority = prio;
	newTask -> state = READY;

	// with this new TCB_t being created, stackPtrAddr should be at highest address
	uint32_t *PSR = (uint32_t*) (newTask -> stackPtrAddr);
	uint32_t *R0 = (uint32_t*) ((newTask -> stackPtrAddr) - (7*sizeof(uint32_t)));
	uint32_t *PC = (uint32_t*) ((newTask -> stackPtrAddr) - sizeof(uint32_t));
	
	// PSR default value is 0x01000000
	*PSR = 0x01000000;
	// PC address of task function
	*PC = (uint32_t) (task);
	// R0 argument to task function
	*R0 = (uint32_t) (arg);

	// the other parts (R4-R11, R1-R3,R12,LR) must exist for the context pop to work
	for (int i = 0; i < 14; i ++) {
		if (i == 8) 
			continue;
		
		uint32_t *currReg = (uint32_t *) (newTask -> stackPtrAddr - (15-i)*sizeof(uint32_t));
		// populate with some value
		*currReg = 0xABCDEF12 + numTasks;
	}
	newTask -> stackPtrAddr -= 15*sizeof(uint32_t);

	// do not enqueue the idle task here
	if (numTasks > 0) {
		enqueue(&PRIORITY_QUEUES[newTask -> priority], newTask);
	}
	
	numTasks ++;
}

void osKernelStart(void) {
	uint32_t *vectorTable = 0x0;
	uint32_t mainStack = vectorTable[0];

	// reset the MSP to the main stack base address
	__set_MSP(mainStack);
	
	// switch from MSP to PSP
	// set bit 1 of control register (SPSEL) to 1
	__set_CONTROL(__get_CONTROL() | 0x2);
	// set register 13 (stack pointer register) to base address of idle task
	__set_PSP(TASKS[0].stackPtrAddr);
	
	printf("Starting kernel");
	// set interupt prioritiees for SysTick and pendsv 
	NVIC_SetPriority(PendSV_IRQn, 0xFF);
	NVIC_SetPriority(SysTick_IRQn, 0x00);
	// invoke the idle task function
	runningTask = &TASKS[0];
	TASKS[0].state = RUNNING;
	runningTaskStackAddr = runningTask -> stackPtrAddr;
	// configure systick
	SysTick_Config(SystemCoreClock/PERIOD);
	t0(NULL);
}

int main(void) {
	if (osKernelInitialize()) {
	
		osThreadNew(t0, NULL, LOWEST);

		#ifdef TEST_CONTEXT_SWITCH
		osThreadNew(t1, NULL, LOWEST);
		osThreadNew(t2, NULL, LOWEST);
		#endif
		
		#ifdef TEST_PRIORITY_QUEUES
		osThreadNew(t1, NULL, MEDIUM);
		osThreadNew(t2, NULL, MEDIUM);
		osThreadNew(t3, NULL, LOW);
		#endif
		
		#ifdef TEST_FPP
		taskParams_t t1p = {.period = 5, .wcet = 4};
		taskParams_t t2p = {.period = 4, .wcet = 2};
		taskParams_t t3p = {.period = 4, .wcet = 1};
		
		taskParameters[1] = t1p;
		taskParameters[2] = t2p;
		taskParameters[3] = t3p;
		
		fppCount[1] = t1p.wcet;
		fppCount[2] = t2p.wcet;
		fppCount[3] = t3p.wcet;
		
		osThreadNew(t1, &taskParameters[1], LOW);
		osThreadNew(t2, &taskParameters[2], MEDIUM);
		osThreadNew(t3, &taskParameters[3], MEDIUM);
		#endif
		
		#ifdef TEST_SEMAPHORES
		osSemaphoreNew(1, 1, NULL);
		osThreadNew(t1, NULL, MEDIUM);
		osThreadNew(t2, NULL, MEDIUM);
		osThreadNew(t3, NULL, MEDIUM);
		osThreadNew(t4, NULL, MEDIUM);
		osThreadNew(t5, NULL, MEDIUM);
		#endif
		
		#ifdef TEST_MUTEXES
		osMutexNew(true);
		osThreadNew(t1, NULL, MEDIUM);
		osThreadNew(t2, NULL, MEDIUM);
		osThreadNew(t3, NULL, MEDIUM);
		osThreadNew(t4, NULL, LOW);
		osThreadNew(t5, NULL, MEDIUM);
		#endif
			
		osKernelStart();
	}
}
