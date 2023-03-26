#include <std_lib.h>
#include <times.h>

int main(void);
void Test_sleep_func(uint32_t val);
void Test_printf_func(void);
void Test_scanf_func(void);
void show_execution_time(uint32_t time_s, uint32_t time_e);

void task0(void); // Toggle LED0
void task1(void); // Toggle LED1
void task2(void); // Toggle LED2
void task3(void); // Toggle LED3

int main(void){     // userland main definition

    while (1){
        uint32_t time_s, time_e;    
        
        time_s = getTime();
        Test_sleep_func(2000);
        time_e = getTime();
        show_execution_time(time_s,time_e);

        time_s = getTime();
        Test_printf_func();
        time_e = getTime();
        show_execution_time(time_s,time_e);

        Test_scanf_func();

        digitalWrite("PA5",1);  // digitalWrite does a write system call
        sleep(2000);
        digitalWrite("PA5",0);
        sleep(2000);
    }
}

void task0(void) // Toggle LED #0
{
    while(1){
        uint32_t time_s, time_e;
        time_s = getTime();
        printf("TASK 0 RUNNING\n");
        digitalWrite("PA5", 1); for(int i=0; i<10000000; i++);
        //sleep(2500);
        digitalWrite("PA5", 0);
        //sleep(2500);
        printf("TASK 0 FINISHED\n"); for(int i=0; i<10000000; i++);
        time_e = getTime();
        printf("TASK 0 RUNNING TIME : %d\n",(time_e-time_s));
    }
}

void task1(void) // Toggle LED #1
{
    while(1){
        uint32_t time_s, time_e;
        time_s = getTime();
        printf("TASK 1 RUNNING\n"); 
        digitalWrite("PA6", 1); for(int i=0; i<30000000; i++);
        //sleep(5000);
        digitalWrite("PA6", 0);
        //sleep(5000);
        printf("TASK 1 FINISHED\n"); for(int i=0; i<30000000; i++);
        time_e = getTime();
        printf("TASK 1 RUNNING TIME : %d\n",(time_e-time_s));
    }
}

void task2(void) // Toggle LED #2
{
    while(1){
        uint32_t time_s, time_e;
        time_s = getTime();
        printf("TASK 2 RUNNING\n");
        digitalWrite("PA7", 1); for(int i=0; i<50000000; i++);
        //sleep(7500);
        digitalWrite("PA7", 0);
        //sleep(7500);
        printf("TASK 2 FINISHED\n"); for(int i=0; i<50000000; i++);
        time_e = getTime();
        printf("TASK 2 RUNNING TIME : %d\n",(time_e-time_s));
    }
}

void task3(void) // Toggle LED #3
{
    while(1){
        uint32_t time_s, time_e;
        time_s = getTime();
        printf("TASK 3 RUNNING\n");
        digitalWrite("PA8", 1); for(int i=0; i<70000000; i++);
        //sleep(10000);
        digitalWrite("PA8", 0);
        //sleep(10000);
        printf("TASK 3 FINISHED\n"); for(int i=0; i<70000000; i++);
        time_e = getTime();
        printf("TASK 3 RUNNING TIME : %d\n",(time_e-time_s));
    }
}

void Test_sleep_func(uint32_t val){
    printf("Sleep Test Before %dms sleep\n",val);
    sleep(val);
    printf("Sleep Test After %dms sleep\n",val);
}

void Test_printf_func(void){
    printf("Printf Test\n");
    printf("Test 1 OK\n");
    printf("Test 2 %d %d\n",6,7);
    int a = 10, b = 93423;
    float f = 324.32;
    printf("Test 3 %d %d %f\n",a,b,f);
    char *string = "String";
    printf("Test %d : %s\n",4,string);
}

void Test_scanf_func(void){
    printf("Scanf Test\n");
    int a, b;
    float f;
    uint8_t string[20];
    printf("Input a b f string\n");
    scanf("%d %d %f %s",&a,&b,&f,string);
    printf("Input : %d %d %f %s\n",a,b,f,string);
}

void show_execution_time(uint32_t time_s, uint32_t time_e){
    uint32_t delta = time_e - time_s;
    printf("CPU Time Spent : %dms\n",delta);
}


