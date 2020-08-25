#include "ecrt.h"
#include "bitop.h"

#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/resource.h>
#include <time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <sys/times.h>
#include <signal.h>
#include <malloc.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <stdio.h>


#define MASK_STATUSWORD 0x417F
#define FLAGS_SWITCHON_DISABLED 0x0140
#define FLAGS_READYTO_SWITCHON 0x0121
#define FLAGS_OPERATION_ENABLE 0x0123
#define FLAGS_ERROR 0x0108
//#define FLAGS_TARGET_REACHED 0x0400

#define MAXON_EPOS 0x000000fb, 0x64400000
#define TASK_FREQUENCY 1000
#define NSEC_PER_SEC (1000000000L)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

const int actuator_count = 2; // number of motors
const int actuator_right_id = 0; // right motor id
const int actuator_left_id = 1; // left motor id

typedef struct _master_info
{
  ec_master_t *master;
  ec_master_state_t master_state;
  ec_domain_t *domain;
  ec_domain_state_t domain_state;
  uint8_t *domain_pd;
  ec_slave_config_t **sc_epos;
  ec_slave_config_state_t *slave_state;
} Master_info;

typedef struct _motor_info
{
  int no;
  int target_pos;
  int act_pos;
  uint16_t status;
} Motor_info;

typedef struct
{
  unsigned int ctrl_word;
  unsigned int target_position;
  unsigned int pos_offset;
  unsigned int vel_offset;
  unsigned int toq_offset;
  unsigned int mod_op;
  unsigned int dig_out;
  unsigned int status_word;
  unsigned int act_velocity;
  unsigned int act_position;
  unsigned int act_toq;
  unsigned int mod_op_dis;
  unsigned int dig_state;
} offset_CSP;

struct timespec timespec_add(struct timespec time1, struct timespec time2);
#if SOD_ACCESS
void read_sdo(void);
void write_sdo(ec_sdo_request_t *sdo, uint8_t *data, size_t size);
#endif

Master_info master_info;
Motor_info *motor_info;
offset_CSP *offset_csp;

/////////////////////////////////////////////
// Local definitions
/////////////////////////////////////////////

void malloc_array()
{
  offset_csp = (offset_CSP *)malloc(sizeof(offset_CSP) * actuator_count);
  motor_info = (Motor_info *)malloc(sizeof(Motor_info) * actuator_count);
  master_info.sc_epos = (ec_slave_config_t **)malloc(sizeof(ec_slave_config_t *) * actuator_count);
  master_info.slave_state = (ec_slave_config_state_t *)malloc(sizeof(ec_slave_config_state_t) * actuator_count);
}

bool is_ecat_bus_ready(int slaveNum)
{
  bool result;
  result = (master_info.slave_state[slaveNum].al_state == 0x08) ? true : false;
  return result;
}

void set_slave_motor_enabled(int slaveNum)
{
  EC_WRITE_U16(master_info.domain_pd + offset_csp[slaveNum].ctrl_word, 0x000F);
}

void set_slave_motor_switchon(int slaveNum)
{
  EC_WRITE_U16(master_info.domain_pd + offset_csp[slaveNum].ctrl_word, 0x0006);
}

void set_slave_motor_reset(int slaveNum)
{
  EC_WRITE_U16(master_info.domain_pd + offset_csp[slaveNum].ctrl_word, 0x0080);
}

bool is_slave_motor_enabled(int slaveNum)
{
  return test_flags(MASK_STATUSWORD, FLAGS_OPERATION_ENABLE, &motor_info[slaveNum].status);
}

bool is_slave_motor_disabled(int slaveNum)
{
  return test_flags(MASK_STATUSWORD, FLAGS_SWITCHON_DISABLED, &motor_info[slaveNum].status);
}

bool is_slave_motor_switchon(int slaveNum)
{
  return test_flags(MASK_STATUSWORD, FLAGS_READYTO_SWITCHON, &motor_info[slaveNum].status);
}

bool is_slave_motor_error(int slaveNum)
{
  return test_flags(MASK_STATUSWORD, FLAGS_ERROR, &motor_info[slaveNum].status);
}

int receive_current_pos(int slave)
{
  motor_info[slave].act_pos = EC_READ_S32(master_info.domain_pd + offset_csp[slave].act_position);
  return motor_info[slave].act_pos;
}

void send_target_pos(int slave)
{
  EC_WRITE_S32(master_info.domain_pd + offset_csp[slave].target_position, motor_info[slave].target_pos);
}

int get_physical_target_pos(int slave)
{
  return motor_info[slave].target_pos;
}

void set_physical_target_pos(int slave, int value)
{
  motor_info[slave].target_pos = value;
}

void init_motor() //using in EcatMgr.cpp
{
  int idx;

  for (idx = 0; idx < actuator_count; ++idx)
  {
    motor_info[idx].no = idx;
    motor_info[idx].target_pos = 0;
    motor_info[idx].act_pos = 0;
    motor_info[idx].status = 0;
  }
}

//////////////////////////////////////////////////////////////

void ecat_up() //using in EcatMgr.cpp
{
  int index;

  master_info.master = ecrt_request_master(0);
  if (!master_info.master)
  {
    exit(EXIT_FAILURE);
  }

  master_info.domain = ecrt_master_create_domain(master_info.master);
  if (!master_info.domain)
  {

    exit(EXIT_FAILURE);
  }

  malloc_array();

  const ec_pdo_entry_reg_t domain1_regs1[] = {
      {0, actuator_right_id, MAXON_EPOS, 0x6040, 0x0, &offset_csp[actuator_right_id].ctrl_word},
      {0, actuator_right_id, MAXON_EPOS, 0x607A, 0x0, &offset_csp[actuator_right_id].target_position},
      {0, actuator_right_id, MAXON_EPOS, 0x60B0, 0x0, &offset_csp[actuator_right_id].pos_offset},
      {0, actuator_right_id, MAXON_EPOS, 0x60B1, 0x0, &offset_csp[actuator_right_id].vel_offset},
      {0, actuator_right_id, MAXON_EPOS, 0x60B2, 0x0, &offset_csp[actuator_right_id].toq_offset},
      {0, actuator_right_id, MAXON_EPOS, 0x6060, 0x0, &offset_csp[actuator_right_id].mod_op},
      {0, actuator_right_id, MAXON_EPOS, 0x2078, 0x1, &offset_csp[actuator_right_id].dig_out},
      {0, actuator_right_id, MAXON_EPOS, 0x6041, 0x0, &offset_csp[actuator_right_id].status_word},
      {0, actuator_right_id, MAXON_EPOS, 0x6077, 0x0, &offset_csp[actuator_right_id].act_toq},
      {0, actuator_right_id, MAXON_EPOS, 0x606C, 0x0, &offset_csp[actuator_right_id].act_velocity},
      {0, actuator_right_id, MAXON_EPOS, 0x6064, 0x0, &offset_csp[actuator_right_id].act_position},
      {0, actuator_right_id, MAXON_EPOS, 0x6061, 0x0, &offset_csp[actuator_right_id].mod_op_dis},
      {0, actuator_right_id, MAXON_EPOS, 0x2071, 0x1, &offset_csp[actuator_right_id].dig_state},
      {}};

  const ec_pdo_entry_reg_t domain1_regs2[] = {
      {0, actuator_left_id, MAXON_EPOS, 0x6040, 0x0, &offset_csp[actuator_left_id].ctrl_word},
      {0, actuator_left_id, MAXON_EPOS, 0x607A, 0x0, &offset_csp[actuator_left_id].target_position},
      {0, actuator_left_id, MAXON_EPOS, 0x60B0, 0x0, &offset_csp[actuator_left_id].pos_offset},
      {0, actuator_left_id, MAXON_EPOS, 0x60B1, 0x0, &offset_csp[actuator_left_id].vel_offset},
      {0, actuator_left_id, MAXON_EPOS, 0x60B2, 0x0, &offset_csp[actuator_left_id].toq_offset},
      {0, actuator_left_id, MAXON_EPOS, 0x6060, 0x0, &offset_csp[actuator_left_id].mod_op},
      {0, actuator_left_id, MAXON_EPOS, 0x2078, 0x1, &offset_csp[actuator_left_id].dig_out},
      {0, actuator_left_id, MAXON_EPOS, 0x6041, 0x0, &offset_csp[actuator_left_id].status_word},
      {0, actuator_left_id, MAXON_EPOS, 0x6077, 0x0, &offset_csp[actuator_left_id].act_toq},
      {0, actuator_left_id, MAXON_EPOS, 0x606C, 0x0, &offset_csp[actuator_left_id].act_velocity},
      {0, actuator_left_id, MAXON_EPOS, 0x6064, 0x0, &offset_csp[actuator_left_id].act_position},
      {0, actuator_left_id, MAXON_EPOS, 0x6061, 0x0, &offset_csp[actuator_left_id].mod_op_dis},
      {0, actuator_left_id, MAXON_EPOS, 0x2071, 0x1, &offset_csp[actuator_left_id].dig_state},
      {}};

  for (index = 0; index < actuator_count; ++index)
  {
    if (!(master_info.sc_epos[index] = ecrt_master_slave_config(master_info.master, 0, index, MAXON_EPOS)))
    {
      fprintf(stderr, "Failed to get slave configuration for EPOS\n");
      exit(EXIT_FAILURE);
    }
  }

  for (index = 0; index < actuator_count; ++index)
  {
    ecrt_slave_config_sdo8(master_info.sc_epos[index], 0x6060, 0x00, 0x08);
    ecrt_slave_config_sdo32(master_info.sc_epos[index], 0x6065, 0x00, 0xFFFFFFFF);
  }

  printf("config sdo\n");
#if CONFIGURE_PDOS
  printf("Configuring PDO\n");

  for (index = 0; index < actuator_count; ++index)
  {
    if (ecrt_slave_config_pdos(master_info.sc_epos[index], EC_END, EPOS3_pdo_1_syncs))
    {
      fprintf(stderr, "Failed to configure PODs\n");
      exit(EXIT_FAILURE);
    }
  }
  printf("Configuring PDO is completed!\n");
#endif

  if (ecrt_domain_reg_pdo_entry_list(master_info.domain, domain1_regs1))
  {
    fprintf(stderr, "PDO entry registration failed1\n");
  }
  if (ecrt_domain_reg_pdo_entry_list(master_info.domain, domain1_regs2))
  {
    fprintf(stderr, "PDO entry registration failed2\n");
  }
  for (index = 0; index < actuator_count; ++index)
  {
    ecrt_slave_config_dc(master_info.sc_epos[index], 0x0300, 1000000, 440000, 0, 0);
  }

  printf("Activating master..\n");

  if (ecrt_master_activate(master_info.master))
  {
    exit(EXIT_FAILURE);
  }

  if (!(master_info.domain_pd = ecrt_domain_data(master_info.domain)))
  {
    exit(EXIT_FAILURE);
  }
}

void ecat_down() //using in EcatMgr.cpp
{
  printf("ecat_down called\n");
  ecrt_master_deactivate(master_info.master);
}

void check_domain_state()
{
  ec_domain_state_t ds;

  ecrt_domain_state(master_info.domain, &ds);

  if (ds.working_counter != master_info.domain_state.working_counter)
    printf("Domain1: WC %u.\n", ds.working_counter);
  if (ds.wc_state != master_info.domain_state.wc_state)
    printf("Domain1: State %u.\n", ds.wc_state);

  master_info.domain_state = ds;
  return;
}

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
  struct timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC)
  {
    result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
  }
  else
  {
    result.tv_sec = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }

  return result;
}

void check_master_state()
{
  ec_master_state_t ms;

  ecrt_master_state(master_info.master, &ms);

  if (ms.slaves_responding != master_info.master_state.slaves_responding)
    printf("%u slave(s).\n", ms.slaves_responding);
  if (ms.al_states != master_info.master_state.al_states)
    printf("AL states: 0x%02X.\n", ms.al_states);
  if (ms.link_up != master_info.master_state.link_up)
    printf("Link is %s.\n", ms.link_up ? "up" : "down");

  master_info.master_state = ms;
}

void postprocessing_ecat() //using in EcatMgr.cpp
{
  struct timespec present_time;
  if (clock_gettime(CLOCK_MONOTONIC, &present_time))
  {
    perror("clock_gettime");
    exit(EXIT_FAILURE);
  }
  ecrt_master_application_time(master_info.master, TIMESPEC2NS(present_time));
  ecrt_master_sync_reference_clock(master_info.master);
  ecrt_master_sync_slave_clocks(master_info.master);

  ecrt_domain_queue(master_info.domain);
  ecrt_master_send(master_info.master);
  usleep(1000000 / TASK_FREQUENCY);
}

void preprocessing_ecat() //using in EcatMgr.cpp
{
  int index;

  ecrt_master_receive(master_info.master);
  ecrt_domain_process(master_info.domain);

  check_domain_state();
  check_master_state();

  for (index = 0; index < actuator_count; ++index)
  {
    ecrt_slave_config_state(master_info.sc_epos[index], &master_info.slave_state[index]);
  }

  for (index = 0; index < actuator_count; ++index)
  {
    motor_info[index].status = EC_READ_U16(master_info.domain_pd + offset_csp[index].status_word);
  }
}

#if SDO_ACCESS
void read_sdo(void)
{
  switch (ecrt_sdo_request_state(sdo))
  {
  case EC_REQUEST_UNUSED:
    ecrt_sdo_request_read(sdo);
    break;
  case EC_REQUEST_BUSY:
    fprintf(stderr, "Still busy\n");
    break;
  case EC_REQUEST_SUCCESS:
    fprintf(stderr, "SDO value : 0x%04X\n", EC_READ_U16(ecrt_sdo_request_data(sdo)));
    ecrt_sdo_request_read(sdo);
    break;
  case EC_REQUEST_ERROR:
    fprintf(stderr, "Failed to read SDO\n");
    ecrt_sdo_request_read(sdo);
    break;
  }
}

void write_sdo(ec_sdo_request_t *sdo, uint8_t *data, size_t size)
{
  switch (ecrt_sdo_request_state(sdo))
  {
  case EC_REQUEST_BUSY:
    ecrt_sdo_request_write(sdo);
    break;
  case EC_REQUEST_UNUSED:
  case EC_REQUEST_SUCCESS:
    if (size == 8)
      EC_WRITE_U64(ecrt_sdo_request_data(sdo), *((uint64_t *)data));
    else if (size == 4)
      EC_WRITE_U32(ecrt_sdo_request_data(sdo), *((uint32_t *)data));
    else if (size == 2)
      EC_WRITE_U16(ecrt_sdo_request_data(sdo), *((uint16_t *)data));
    else
      EC_WRITE_U8(ecrt_sdo_request_data(sdo), *((uint8_t *)data));
    ecrt_sdo_request_write(sdo);
  case EC_REUQEST_ERROR:
    fprintf(stderr, "Failed to write SDO! data:[0x%X], size:[%d]\n", *data, size);
    ecrt_sdo_request_write(sdo);
    break;
  }
}

#endif
