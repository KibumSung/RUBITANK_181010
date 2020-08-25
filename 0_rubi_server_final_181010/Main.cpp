#include <stdio.h>
#include <iostream>
#include "keyboard.h"
#include <string.h>
#include "HybridAutomata.h"
extern int ecat_state;
extern void init_state_machine_for_ECAT();
extern HybridAutomata *HA_ecatmgr; //Main.cpp is using
extern unsigned int before_pre_ecat_state;
extern int receive_current_pos(int slave);
extern int get_physical_target_pos(int slave);
extern void set_physical_target_pos(int slave, int value);
extern const int actuator_count = 2; // number of motors
extern const int actuator_right_id = 0; // rigth motor id
extern const int actuator_left_id = 1; // left motor id

char ecat[10];
char ecat_pdo[10];

void change_int_2_str()
{
    switch(ecat_state)
    {
        case 0: strcpy(ecat,"ECAT_START\n");break;
        case 1: strcpy(ecat,"ECAT_UP\n");break;
        case 2: strcpy(ecat,"ECAT_ON\n");break;
        case 3: strcpy(ecat,"ECAT_OFF\n");break;
        case 4: strcpy(ecat,"ECAT_DOWN\n");break;
    }
    switch(before_pre_ecat_state)
    {
        case 0: strcpy(ecat_pdo,"PRE_ECAT_START\n");break;
        case 1: strcpy(ecat_pdo,"PRE_ECAT_ERROR\n");break;
        case 2: strcpy(ecat_pdo,"PRE_ECAT_BUS_ON\n");break;
        case 3: strcpy(ecat_pdo,"PRE_ECAT_PDO_MAPPING\n");break;
        case 4: strcpy(ecat_pdo,"PRE_ECAT_FINISH\n");break;
    }
}
void print_manual()
{

    printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
    printf("$$              How to use(Don't hit enter)         $$\n");
    printf("$$--------------------------------------------------$$\n");
    printf("$$  ECAT UP = 1/ ON = 2/ OFF = 3/ DOWN = 4/         $$\n");
    printf("$$  Debug = D/ -> Use when U want to move motor     $$\n");
    printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
}
void print_state_status()
{
    change_int_2_str();
    printf("\n##################################\n");
    printf("##  EtherCAT State = %s",ecat);
    printf("##  EtherCAT PDO Mapping State = %s",ecat_pdo);
    printf("##################################\n\n");
}
int main()
{
    init_state_machine_for_ECAT();
    char key_in=' ';
    int right_value=0;
	int left_value=0;
    while(1)
    {
        print_manual();
        key_in = kbhit();
        if(key_in == '\n');
        printf("\n\tkeyboard input = %c.\n\n", key_in);
        if (key_in > '0' && key_in < '5')
        {
            ecat_state = key_in - '0';
            HA_ecatmgr->operate();
        }
        else if (key_in == 'D')
        {
            char slave;
            int value;
            printf("Go : G\nBack : B\nRight : R\n Left : L\n");
            scanf("%c", &slave);

            switch (slave)
            {
            case 'G':
                printf("Go\n");
                right_value += 20000;
				left_value += 20000;
				value = right_value;
                set_physical_target_pos(actuator_right_id,value);
				value = left_value;
				set_physical_target_pos(actuator_left_id,value);
                break;
            case 'B':
                printf("Back\n");
                right_value -= 20000;
				left_value -= 20000;
				value = right_value;
                set_physical_target_pos(actuator_right_id,value);
				value = left_value;
				set_physical_target_pos(actuator_left_id,value);
                break;
			case 'R':
				printf("Turn right\n");
				right_value -= 15000;
				left_value += 15000;
				value = right_value;
				set_physical_target_pos(actuator_right_id,value);
				value = left_value;
				set_physical_target_pos(actuator_left_id,value);
				break;
			case 'L':
				printf("Turn left\n");
				right_value += 15000;
				left_value -= 15000;
				value = right_value;
				set_physical_target_pos(actuator_right_id,value);
				value = left_value;
				set_physical_target_pos(actuator_left_id,value);
				break;
            default:
                printf("Unknown key");
                break;
            }
            printf("##  Gear Motor Target Pos = %d, Current Pos = %d\n",receive_current_pos(actuator_right_id)
    ,get_physical_target_pos(actuator_right_id));
    printf("##  Wheel Motor Target Pos = %d, Current Pos = %d\n",receive_current_pos(actuator_left_id)
    ,get_physical_target_pos(actuator_left_id));
        }
        else printf("Unknown Key\n");

        print_state_status();
    }
}
