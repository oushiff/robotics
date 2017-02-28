/*============================================================================
==============================================================================
                      
                              min_jerk_task.cpp
 
==============================================================================
Remarks:

      sekeleton to create the sample task

============================================================================*/

// system headers
#include "SL_system_headers.h"

/* SL includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"

// defines

// local variables
static double      start_time = 0.0;
static SL_DJstate  target[N_DOFS+1];
static double      delta_t = 0.01;
static double      duration = 1.0;
static double      time_to_go;
static int count = 0;
static int mode = 2; //[1] original mode  [2] question-e mode

// global functions 
extern "C" void
add_min_jerk_task( void );

// local functions
static int  init_min_jerk_task(void);
static int  run_min_jerk_task(void);
static int  change_min_jerk_task(void);

static int 
min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
            double t_togo, double dt,
            double *x_next, double *xd_next, double *xdd_next);


/*****************************************************************************
******************************************************************************
Function Name   : add_min_jerk_task
Date        : Feb 1999
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_min_jerk_task( void )
{
  int i, j;
  
  addTask("Min Jerk Task", init_min_jerk_task, 
      run_min_jerk_task, change_min_jerk_task);

}    



/*****************************************************************************
******************************************************************************
  Function Name : init_min_jerk_task
  Date      : Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_min_jerk_task(void)
{
  int j, i;
  int ans;
  static int firsttime = TRUE;
  
  if (firsttime){
    firsttime = FALSE;
  }

  // prepare going to the default posture
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  for (i=1; i<=N_DOFS; i++)
    target[i] = joint_default_state[i];

  // go to the target using inverse dynamics (ID)
  if (!go_target_wait_ID(target)) 
    return FALSE;

  // re-use the variable target for our min-jerk movement: only the right arm moves
  target[R_SFE].th += 0.4;
  target[R_SAA].th -= 0.4;
  target[R_EB].th  -= 0.5;



  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  // only go when user really types the right thing
  if (ans != 1) 
    return FALSE;

  start_time = task_servo_time;
  printf("start time = %.3f, task_servo_time = %.3f\n", 
     start_time, task_servo_time);

  // start data collection
  scd();

  // time to go
  time_to_go = duration;

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name : run_min_jerk_task
  Date      : Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_min_jerk_task(void)
{
  int j, i;

  double task_time;

  // ******************************************
  // NOTE: all array indices start with 1 in SL
  // ******************************************

  task_time = task_servo_time - start_time;

  // compute the update for the desired states
  for (i=1; i<=N_DOFS; ++i) {
    min_jerk_next_step(joint_des_state[i].th,
               joint_des_state[i].thd,
               joint_des_state[i].thdd,
               target[i].th,
               target[i].thd,
               target[i].thdd,
               time_to_go,
               delta_t,
               &(joint_des_state[i].th),
               &(joint_des_state[i].thd),
               &(joint_des_state[i].thdd));
  }

  // compute inverse dynamics torques
  SL_InvDynNE(joint_state,joint_des_state,endeff,&base_state,&base_orient);
  
  // decrement time to go
  time_to_go -= delta_t;
  
  if ( time_to_go <= 0) {
    if (count == 0) {
      time_to_go = duration;
      target[L_SFE].th -= 0.2;
      run_min_jerk_task();
      count++;

    } 
    else if (count == 1) {
      time_to_go = duration;
      target[L_SAA].th -= 0.2;
      target[L_EB].th -= 0.05;
      run_min_jerk_task();
      count++;
    } 
    else if (count == 2) {
      time_to_go = duration;
      target[L_SFE].th += 0.2;
      run_min_jerk_task();
      count++;
    } 
    else if (count == 3) {
      time_to_go = duration;
      target[L_SFE] = joint_default_state[L_SFE];
      target[L_SAA] = joint_default_state[L_SAA];
      target[L_EB] = joint_default_state[L_EB];
      run_min_jerk_task();
      count++;
    } 
    else {
      mode = 0;
      count = 0;
      freeze();
    }
  }

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name : change_min_jerk_task
  Date      : Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_min_jerk_task(void)
{
  int    ivar;
  double dvar;

  get_int("This is how to enter an integer variable",ivar,&ivar);
  get_double("This is how to enter a double variable",dvar,&dvar);

  return TRUE;

}





static double getX(double xf, double x, double xd, double xdd, double t) {
  double res1 = 2*xf - 2*x - 2*t*xd - xdd*t*t;
  double res2 = 2.0 * t*t*t;
  return res1 / res2;
}

static double getY(double xfd, double xd, double xdd, double t) {
  double res1 = xfd - xd - t*xdd;
  double res2 = t*t;
  return res1 / res2;
}

static double getZ(double xfdd, double xdd, double t) {
  double res1 = xfdd - xdd;
  double res2 = t;
  return res1 / res2;
}

static double getC3(double x, double xd, double xdd, double xf, double xfd, double xfdd, double t) {
  double res1 = 20*getX(xf, x, xd, xdd, t) - 8*getY(xfd, xd, xdd, t) + getZ(xfdd, xdd, t); 
  return res1 / 2.0;
}

static double get_xddd(double x, double xd, double xdd, double xf, double xfd, double xfdd, double t) {
  return 6 * getC3(x, xd, xdd, xf, xfd, xfdd, t) ;
}
//

static double get_xdd2(double x, double xd, double xf, double t) {
  int alpha = 25;
  int beta = 6;
  double res1 = alpha * (beta * (xf - x) - xd);
  return res1 / t;
}

/*!*****************************************************************************
 *******************************************************************************
\note  min_jerk_next_step
\date  April 2014
   
\remarks 

Given the time to go, the current state is updated to the next state
using min jerk splines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]          x,xd,xdd : the current state, vel, acceleration
 \param[in]          t,td,tdd : the target state, vel, acceleration
 \param[in]          t_togo   : time to go until target is reached
 \param[in]          dt       : time increment
 \param[in]          x_next,xd_next,xdd_next : the next state after dt

 ******************************************************************************/
static int 
min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
            double t_togo, double dt,
            double *x_next, double *xd_next, double *xdd_next)

{

  // your code goes here ...
  if (mode == 2) {
    *xdd_next = get_xdd2(x, xd, t, t_togo);
  } else {
    double xddd = get_xddd(x, xd, xdd, t, td, tdd, t_togo);
    *xdd_next = xdd + xddd * dt;
  }
  *xd_next = xd + *xdd_next * dt;
  *x_next = x + *xd_next * dt;
  return TRUE;
}











