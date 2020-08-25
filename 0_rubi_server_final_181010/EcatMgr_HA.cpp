// First level of HybridAutomata

#include "HybridAutomata.h"
#include <pthread.h>
#include <sched.h>
#include <stdlib.h>
#include <string>
#include <map>

/////////////////////////////////////////////
// Imported definitions from Global_DB.cpp
/////////////////////////////////////////////

unsigned int ecat_state;
extern const int actuator_count;
/////////////////////////////////////////////
// Imported definitions from other HybridAutomata
/////////////////////////////////////////////


/////////////////////////////////////////////
// Imported definitions from ecat.c
/////////////////////////////////////////////

extern void set_slave_motor_enabled(int slaveNum);
extern void set_slave_motor_switchon(int slaveNum);
extern void set_slave_motor_reset(int slaveNum);
extern bool is_ecat_bus_ready(int slaveNum);
extern bool is_slave_motor_disabled(int slaveNum);
extern bool is_slave_motor_switchon(int slaveNum);
extern bool is_slave_motor_enabled(int slaveNum);
extern bool is_slave_motor_error(int slaveNum);

extern void ecat_up();
extern void ecat_down();
extern void init_motor();
extern void preprocessing_ecat();
extern void postprocessing_ecat();
extern int receive_current_pos(int slave);
extern void send_target_pos(int slave);
/////////////////////////////////////////////
// Exported definitions
/////////////////////////////////////////////

HybridAutomata *HA_ecatmgr; //Main.cpp is using

/////////////////////////////////////////////
// Local definitions
/////////////////////////////////////////////

////////////for ecat_error///////
unsigned int err_slave_idx;

enum
{
    ECAT_START,
    ECAT_UP,
    ECAT_ON,
    ECAT_OFF,
    ECAT_DOWN,
    ECAT_FINISH
};

enum
{
    PRE_ECAT_START,
    PRE_ECAT_ERROR,
    PRE_ECAT_BUS_ON,
    PRE_ECAT_PDO_MAPPING,
    PRE_ECAT_FINISH
};
pthread_t cyclic_thread;
unsigned int before_pre_ecat_state;

HybridAutomata *HA_prepareecat;
void init_state_machine_for_Preparing_ECAT();

void pre_ecat_error()
{
  //cout << "\t\tecat_error" <<endl;
  set_slave_motor_enabled(err_slave_idx);
  preprocessing_ecat();
  set_slave_motor_reset(err_slave_idx);
}
void pre_ecat_bus_on()
{
  //cout << "\t\tecat_bus_on" <<endl;
}

void pre_ecat_pdo_mapping()
{
  //cout << "\t\tecat_pdo_mapping" <<endl;
  //cout<< " count : "<<cnt++<<endl;
}

void *cyclic_task(void *)
{
  struct sched_param sparam;
  sparam.sched_priority = 49;
  sched_setscheduler(0, SCHED_FIFO, &sparam);

  while (1)
  {
    preprocessing_ecat();
    //cout << "curState : "<< HA_behavior->curState<<endl;
    //cout<< "ecat_var[ECAT_BEHAVIOR_STATE].value :" << ecat_var[ECAT_BEHAVIOR_STATE].value << endl;
    for (int index = 0; index < actuator_count; ++index)
  {
    receive_current_pos(index);
    send_target_pos(index);
  }

    postprocessing_ecat();

  } //end of while

  return NULL;
}

void ecat_on()
{
  init_state_machine_for_Preparing_ECAT();

  while(1)
  {
    preprocessing_ecat();
    HA_prepareecat->operate();
    postprocessing_ecat();
    if(HA_prepareecat->curState == PRE_ECAT_FINISH)
    {
      HA_prepareecat->curState = PRE_ECAT_START;
      break;
    }
  }

  init_motor();

  if (pthread_create(&cyclic_thread, 0, cyclic_task, NULL))
  {
    printf("Thread Err\n");
    exit(EXIT_FAILURE);
  }
}

void ecat_off()
{
  printf("ecat_off called\n");
  pthread_cancel(cyclic_thread);
  pthread_join(cyclic_thread, NULL);
}

void pre_ecat_finish()
{
  return;
}
class COND_ALL_2_ECAT_ERROR : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    for (int index = 0; index < actuator_count; ++index)
    {
      if (is_slave_motor_error(index))
      {
        err_slave_idx = index;
        before_pre_ecat_state = HA->curState;
        //cout << "condition : COND_ALL_2_ECAT_ERROR" << endl;
        return true;
      }
    }
    return false;
  }
};


class COND_ECAT_ERROR_2_ECAT_BUS_ON : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (before_pre_ecat_state == PRE_ECAT_BUS_ON)
    {
      // cout << "condition : COND_ECAT_ERROR_2_ECAT_BUS_ON" << endl;
      return true;
    }
    return false;
  }
};

class COND_ECAT_ERROR_2_ECAT_PDO_MAPPING : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (before_pre_ecat_state == PRE_ECAT_PDO_MAPPING)
    {
      //  cout << "condition : COND_ECAT_ERROR_2_ECAT_PDO_MAPPING" << endl;
      return true;
    }
    return false;
  }
};

//////////////////////////////////////////////////
// 2. Conditions : ECAT_BUS_ON -> *
//    ECAT_BUS_ON State is Init State.
//////////////////////////////////////////////////

class COND_ECAT_BUS_ON_2_ECAT_BUS_ON : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    for (int index = 0; index < actuator_count; ++index)
    {
      if (is_ecat_bus_ready(index)==false)
      {
        //cout << "condition : COND_ECAT_BUS_ON_2_ECAT_BUS_ON" << endl;
        return true;
      }
    }
  
  return false;
  }
};

class COND_ECAT_BUS_ON_2_ECAT_PDO_MAPPING : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    int cnt = 0;
    for (int index = 0; index < actuator_count; ++index)
    {
      if (is_ecat_bus_ready(index)==true)
      {
        cnt++;
      }
    }
    if (cnt == actuator_count)
    {
      cout << "condition : COND_ECAT_BUS_ON_2_ECAT_PDO_MAPPING" << endl;
      return true;
    }
  
  //cout << "\tcount : "<<count<<endl;
  //cout << "condition : COND_ECAT_BUS_ON_2_ECAT_PDO_MAPPING" << endl;
  return false;
  }
};

//////////////////////////////////////////////////
// 3. Conditions : ECAT_PDO_MAPPING -> *
//////////////////////////////////////////////////

class COND_ECAT_PDO_MAPPING_2_ECAT_PDO_MAPPING : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    bool result = false;

    for (int index = 0; index < actuator_count; ++index)
    {
      if (is_slave_motor_disabled(index))
      {
        set_slave_motor_switchon(index);
        result = true;
        //cout << "IN IF index : "<< index << " "<<result<< endl;
      }
      else if (is_slave_motor_switchon(index))
      {
        set_slave_motor_enabled(index);
        result = true;
        //cout << "IN ELSE IF index : "<< index << " "<<result << endl;
      }
  
    }
  
  return result;
  }
};

class COND_ECAT_PDO_MAPPING_2_FINISH : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    for (int index = 0; index < actuator_count; ++index)
    {
      
      if (is_slave_motor_enabled(index))
      {
        switch (index)
        {
        /*  //case GEAR_SLAVE :
          //sprintf(log_buf, "Gear Slave On\n");
          //write(fd, log_buf, strlen(log_buf));
          //break;
        case HANDLE_SLAVE:
          //sprintf(log_buf, "Handle Slave On\n");
          printf("Handle Slave On\n");
          // write(motor_fd, log_buf, strlen(log_buf));
          break;
        case ACCEL_SLAVE:
          //sprintf(log_buf, "Accel Slave On\n");
          printf("Accel Slave On\n");
          // write(motor_fd, log_buf, strlen(log_buf));
          break;
        case BRAKE_SLAVE:
          //sprintf(log_buf, "Brake Slave On\n");
          printf("Brake Slave On\n");
          // write(motor_fd, log_buf, strlen(log_buf));
          break;
          */
        }
      }
      //else return false;
    }
    cout << "condition : COND_ECAT_PDO_MAPPING_2_FINISH" << endl;

    // motor_info[ACCEL_SLAVE].target_pos = 0;
    // motor_info[BRAKE_SLAVE].target_pos = 0;
    // motor_info[GEAR_SLAVE].target_pos = 0;
    // motor_info[HANDLE_SLAVEHANDLE_SLAVE].target_pos = 0;
    //printf("get_slave_motor_pos : %d\n", get_slave_motor_pos(HANDLE_SLAVE));
    return true;
 
  }
};

class COND_ECAT_UP_2_ON : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (ecat_state == ECAT_ON)
      return true;
    
    else
      return false;
  }
};

class COND_ECAT_ON_2_OFF : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (ecat_state == ECAT_OFF)
      return true;
    else
      return false;
  }
};

class COND_ECAT_OFF_2_DOWN : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (ecat_state == ECAT_DOWN)
      return true;
    else
      return false;
  }
};

/////////////////////////////////////////////
// Exported definitions
/////////////////////////////////////////////

void init_state_machine_for_Preparing_ECAT()
{
  HA_prepareecat = new HybridAutomata(PRE_ECAT_START, PRE_ECAT_FINISH);
  HA_prepareecat->setCondition(PRE_ECAT_START, NULL, PRE_ECAT_BUS_ON);

  HA_prepareecat->setState(PRE_ECAT_ERROR, pre_ecat_error);
  HA_prepareecat->setState(PRE_ECAT_BUS_ON, pre_ecat_bus_on);
  HA_prepareecat->setState(PRE_ECAT_PDO_MAPPING, pre_ecat_pdo_mapping);
  HA_prepareecat->setState(PRE_ECAT_FINISH, pre_ecat_finish);
  
  ////////////////////// conditions * -> ecat_error  //////////////
  COND_ALL_2_ECAT_ERROR *cond_all_2_error = new COND_ALL_2_ECAT_ERROR();

  HA_prepareecat->setCondition(PRE_ECAT_BUS_ON, cond_all_2_error, PRE_ECAT_ERROR);
  HA_prepareecat->setCondition(PRE_ECAT_PDO_MAPPING, cond_all_2_error, PRE_ECAT_ERROR);

  ////////////////////// conditions ecat_error -> *  //////////////
  COND_ECAT_ERROR_2_ECAT_BUS_ON *cond_error_2_bus_on = new COND_ECAT_ERROR_2_ECAT_BUS_ON();
  COND_ECAT_ERROR_2_ECAT_PDO_MAPPING *cond_error_2_pdo_mapping = new COND_ECAT_ERROR_2_ECAT_PDO_MAPPING();

  HA_prepareecat->setCondition(PRE_ECAT_ERROR, cond_error_2_bus_on, PRE_ECAT_BUS_ON);
  HA_prepareecat->setCondition(PRE_ECAT_ERROR, cond_error_2_pdo_mapping, PRE_ECAT_PDO_MAPPING);

  ////////////////////// conditions of ecat_bus_on -> */////////////////
  COND_ECAT_BUS_ON_2_ECAT_BUS_ON *cond_bus_on_2_bus_on = new COND_ECAT_BUS_ON_2_ECAT_BUS_ON();
  COND_ECAT_BUS_ON_2_ECAT_PDO_MAPPING *cond_bus_on_2_pdo_mapping = new COND_ECAT_BUS_ON_2_ECAT_PDO_MAPPING();

  HA_prepareecat->setCondition(PRE_ECAT_BUS_ON, cond_bus_on_2_bus_on, PRE_ECAT_BUS_ON);
  HA_prepareecat->setCondition(PRE_ECAT_BUS_ON, cond_bus_on_2_pdo_mapping, PRE_ECAT_PDO_MAPPING);

  ////////////////////// conditions of ecat_pdo_mapping -> */////////////////
  COND_ECAT_PDO_MAPPING_2_ECAT_PDO_MAPPING *cond_pdo_mapping_2_pdo_mapping = new COND_ECAT_PDO_MAPPING_2_ECAT_PDO_MAPPING();
  COND_ECAT_PDO_MAPPING_2_FINISH *cond_pdo_mapping_2_finish = new COND_ECAT_PDO_MAPPING_2_FINISH();
  
  HA_prepareecat->setCondition(PRE_ECAT_PDO_MAPPING, cond_pdo_mapping_2_pdo_mapping, PRE_ECAT_PDO_MAPPING);
  HA_prepareecat->setCondition(PRE_ECAT_PDO_MAPPING, cond_pdo_mapping_2_finish, PRE_ECAT_FINISH);
}

void init_state_machine_for_ECAT()
{
  cout << "in init_state_machine_for_ECAT func()" << endl;
  HA_ecatmgr = new HybridAutomata(ECAT_START, ECAT_FINISH);

  HA_ecatmgr->setState(ECAT_UP, ecat_up);
  HA_ecatmgr->setState(ECAT_ON, ecat_on);
  HA_ecatmgr->setState(ECAT_OFF, ecat_off);
  HA_ecatmgr->setState(ECAT_DOWN, ecat_down);

  COND_ECAT_UP_2_ON *cond_up_2_on = new COND_ECAT_UP_2_ON();
  COND_ECAT_ON_2_OFF *cond_on_2_off = new COND_ECAT_ON_2_OFF();
  COND_ECAT_OFF_2_DOWN *cond_off_2_down = new COND_ECAT_OFF_2_DOWN();

  HA_ecatmgr->setCondition(ECAT_START, NULL, ECAT_UP);
  HA_ecatmgr->setCondition(ECAT_UP, cond_up_2_on, ECAT_ON);
  HA_ecatmgr->setCondition(ECAT_ON, cond_on_2_off, ECAT_OFF);
  HA_ecatmgr->setCondition(ECAT_OFF, cond_off_2_down, ECAT_DOWN);
  HA_ecatmgr->setCondition(ECAT_DOWN, NULL, ECAT_FINISH);
}
