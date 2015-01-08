/*
 * sfuntmpl_basic.c: Basic 'C' template for a level 2 S-function.
 *
 *  -------------------------------------------------------------------------
 *  | See matlabroot/simulink/src/sfuntmpl_doc.c for a more detailed template |
 *  -------------------------------------------------------------------------
 *
 * Copyright 1990-2002 The MathWorks, Inc.
 * $Revision: 1.27.4.2 $
 */


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  fuzzy_ctrl
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

 float minimum(float num[6]);
 float maxi(float min_val,float max_val);

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* See sfuntmpl_doc.c for more details on the macros below */

    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 6);
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 2);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T *u = (const real_T*) ssGetInputPortSignal(S,0);
    real_T       *y = ssGetOutputPortSignal(S,0);
   /* if(u[0]>5)
    {
      y[0] = 2*u[0];
    }
    else
    {
         y[0] = u[1];
    }
    */
    //
    //printf("enter height \n ");
    
    float T_Distance=u[0];
    float T_Direction=u[1];
    float L_Obstacle=u[2];
    float F_Obstacle=u[3];
    float R_Obstacle=u[4];
    float B_Obstacle=u[5];
    
  // Outputs
    float L_MotorSpeed=0.0;
    float R_MotorSpeed=0.0;
  //Lingustic variable
    float T_dis_mf[3]={0.0,0.0,0.0};
    float T_dir_mf[4]={0.0,0.0,0.0,0.0};
    float obs_mf[4][3]={{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    float MotorSpeed_mf[2][7]={{0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};
    //Data
    float T_DS[3]={0.0,7.5,15.0};
    int T_DI[5]={0,90,180,270,360};
    float OBS[3]={0.0,10.0,20.0};
    float MS[9]={-266.67,-200.0,-133.34,-66.67,0.0,66.67,133.34,200.0,266.67};
   //general
   float mini=0.0;
   float maximum=0.0;
   float temp[6];
   // defuzzification variable
   float Areas[2][7];
   float bases[2];
   //
    int k;
    int l;
    int r;
    int f;
    int b;
    int d;
    
   //Fuzzificaion
   //******* Target distance *********
   {
       if(T_Distance>=T_DS[0] && T_Distance<T_DS[1])
   {
   T_dis_mf[0]=((0.0-1.0)/(T_DS[1]-T_DS[0]))*(T_Distance-T_DS[0])+1.0;
   T_dis_mf[1]=((1.0-0.0)/(T_DS[1]-T_DS[0]))*(T_Distance-T_DS[0])+0.0;
   }
   else
    if(T_Distance>=T_DS[1] && T_Distance<T_DS[2])
   {
    T_dis_mf[1]=((0.0-1.0)/(T_DS[2]-T_DS[1]))*(T_Distance-T_DS[1])+1.0;
    T_dis_mf[2]=((1.0-0.0)/(T_DS[2]-T_DS[1]))*(T_Distance-T_DS[1])+0.0;
   }
   else
    if(T_Distance>=T_DS[2])
   {
       T_dis_mf[2]=1.0;
   }
   else
   {
       T_dis_mf[0]=0.0;
       T_dis_mf[1]=0.0;
       T_dis_mf[2]=0.0;
   }
   // ********* Target direction ************
   if(T_Direction>=T_DI[0] && T_Direction<T_DI[1])
   {
       T_dir_mf[0]=((0.0-1.0)/(T_DI[1]-T_DI[0]))*(T_Direction-T_DI[0])+1.0;
       T_dir_mf[1]=((1.0-0.0)/(T_DI[1]-T_DI[0]))*(T_Direction-T_DI[0])+0.0;
   }
   else
    if(T_Direction>=T_DI[1] && T_Direction<T_DI[2])
   {
       T_dir_mf[1]=((0.0-1.0)/(T_DI[2]-T_DI[1]))*(T_Direction-T_DI[1])+1.0;
       T_dir_mf[2]=((1.0-0.0)/(T_DI[2]-T_DI[1]))*(T_Direction-T_DI[1])+0.0;
   }
   else
    if(T_Direction>=T_DI[2] && T_Direction<T_DI[3])
   {
       T_dir_mf[2]=((0.0-1.0)/(T_DI[3]-T_DI[2]))*(T_Direction-T_DI[2])+1.0;
       T_dir_mf[3]=((1.0-0.0)/(T_DI[3]-T_DI[2]))*(T_Direction-T_DI[2])+0.0;
   }
   else
    if(T_Direction>=T_DI[3] && T_Direction<T_DI[4])
   {
       T_dir_mf[3]=((0.0-1.0)/(T_DI[4]-T_DI[3]))*(T_Direction-T_DI[3])+1.0;
       T_dir_mf[0]=((1.0-0.0)/(T_DI[4]-T_DI[3]))*(T_Direction-T_DI[3])+0.0;
   }
   else
   {
      T_dir_mf[0]=0.0;
      T_dir_mf[1]=0.0;
      T_dir_mf[2]=0.0;
      T_dir_mf[3]=0.0;
   }
   // ******* Obstacles **********
   //Left
   if(L_Obstacle>=OBS[0] && L_Obstacle<OBS[1])
   {
     obs_mf[0][0]=((0.0-1.0)/(OBS[1]-OBS[0]))*(L_Obstacle-OBS[0])+1.0;
     obs_mf[0][1]=((1.0-0.0)/(OBS[1]-OBS[0]))*(L_Obstacle-OBS[0])+0.0;
   }
   else
    if(L_Obstacle>=OBS[1] && L_Obstacle<OBS[2])
   {
     obs_mf[0][1]=((0.0-1.0)/(OBS[2]-OBS[1]))*(L_Obstacle-OBS[1])+1.0;
     obs_mf[0][2]=((1.0-0.0)/(OBS[2]-OBS[1]))*(L_Obstacle-OBS[1])+0.0;
   }
   else
    if(L_Obstacle>=OBS[2])
   {
       obs_mf[0][2]=1.0;
   }
   else
   {
       obs_mf[0][0]=0.0;
       obs_mf[0][1]=0.0;
       obs_mf[0][2]=0.0;
   }
   //Front
   if(F_Obstacle>=OBS[0] && F_Obstacle<OBS[1])
   {
     obs_mf[1][0]=((0.0-1.0)/(OBS[1]-OBS[0]))*(F_Obstacle-OBS[0])+1.0;
     obs_mf[1][1]=((1.0-0.0)/(OBS[1]-OBS[0]))*(F_Obstacle-OBS[0])+0.0;
   }
   else
    if(F_Obstacle>=OBS[1] && F_Obstacle<OBS[2])
   {
     obs_mf[1][1]=((0.0-1.0)/(OBS[2]-OBS[1]))*(F_Obstacle-OBS[1])+1.0;
     obs_mf[1][2]=((1.0-0.0)/(OBS[2]-OBS[1]))*(F_Obstacle-OBS[1])+0.0;
   }
   else
    if(F_Obstacle>=OBS[2])
   {
       obs_mf[1][2]=1.0;
   }
   else
   {
       obs_mf[1][0]=0.0;
       obs_mf[1][1]=0.0;
       obs_mf[1][2]=0.0;
   }
    // Right
    if(R_Obstacle>=OBS[0] && R_Obstacle<OBS[1])
   {
     obs_mf[2][0]=((0.0-1.0)/(OBS[1]-OBS[0]))*(R_Obstacle-OBS[0])+1.0;
     obs_mf[2][1]=((1.0-0.0)/(OBS[1]-OBS[0]))*(R_Obstacle-OBS[0])+0.0;
   }
   else
    if(R_Obstacle>=OBS[1] && R_Obstacle<OBS[2])
   {
     obs_mf[2][1]=((0.0-1.0)/(OBS[2]-OBS[1]))*(R_Obstacle-OBS[1])+1.0;
     obs_mf[2][2]=((1.0-0.0)/(OBS[2]-OBS[1]))*(R_Obstacle-OBS[1])+0.0;
   }
   else
    if(R_Obstacle>=OBS[2])
   {
       obs_mf[2][2]=1.0;
   }
   else
   {
       obs_mf[2][0]=0.0;
       obs_mf[2][1]=0.0;
       obs_mf[2][2]=0.0;
   }
   //Back
   if(B_Obstacle>=OBS[0] && B_Obstacle<OBS[1])
   {
     obs_mf[3][0]=((0.0-1.0)/(OBS[1]-OBS[0]))*(B_Obstacle-OBS[0])+1.0;
     obs_mf[3][1]=((1.0-0.0)/(OBS[1]-OBS[0]))*(B_Obstacle-OBS[0])+0.0;
   }
   else
    if(B_Obstacle>=OBS[1] && B_Obstacle<OBS[2])
   {
     obs_mf[3][1]=((0.0-1.0)/(OBS[2]-OBS[1]))*(B_Obstacle-OBS[1])+1.0;
     obs_mf[3][2]=((1.0-0.0)/(OBS[2]-OBS[1]))*(B_Obstacle-OBS[1])+0.0;
   }
   else
    if(B_Obstacle>=OBS[2])
   {
       obs_mf[3][2]=1.0;
   }
   else
   {
       obs_mf[3][0]=0.0;
       obs_mf[3][1]=0.0;
       obs_mf[3][2]=0.0;
   }
 printf(" Target Distance mf ={%f, %f, %f }\n",T_dis_mf[0],T_dis_mf[1],T_dis_mf[2]);
 printf("Target Direction mf ={%f, %f, %f, %f}  \n",T_dir_mf[0],T_dir_mf[1],T_dir_mf[2],T_dir_mf[3]);
 printf("      Left Obstacle ={%f, %f, %f}  \n",obs_mf[0][0],obs_mf[0][1],obs_mf[0][2]);
 printf("     Front Obstacle ={%f, %f, %f}  \n",obs_mf[1][0],obs_mf[1][1],obs_mf[1][2]);
 printf("     Right Obstacle ={%f, %f, %f}  \n",obs_mf[2][0],obs_mf[2][1],obs_mf[2][2]);
 printf("       Back Ostacle ={%f, %f, %f}  \n",obs_mf[3][0],obs_mf[3][1],obs_mf[3][2]);
   }

 //*************** Inference System *************************
 //*******Left Motor ******************
 //*******Forward Fast ***************************
 {
 temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
 mini=minimum(temp);
 maximum=mini;
 temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
 mini=minimum(temp);
maximum=maxi(mini,maximum);
 temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
 mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
	
MotorSpeed_mf[0][6]=maximum;
printf("*********** Left Motor *********** \n");
printf("Left Motor Forward_F = %f \n",MotorSpeed_mf[0][6]);
 }

//********* Forward Medium ***********************
{
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
MotorSpeed_mf[0][5]=maximum;
printf("Left Motor Forward_M = %f \n",MotorSpeed_mf[0][5]);
}

//********* Forward Slow *************************
{

temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
	
 MotorSpeed_mf[0][4]=maximum;
 printf("Left Motor Forward_S = %f \n",MotorSpeed_mf[0][4]);
}
// ************* Stop ****************************
{

    temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);

for(k=0;k<4;k++)
{
   for(l=0;l<3;l++)
   {
       for(r=0;r<3;r++)
       {
           for(f=0;f<3;f++)
           {
               for(b=0;b<3;b++)
               {
   temp[0]=T_dis_mf[0];	temp[1]=T_dir_mf[k];	temp[2]=obs_mf[0][l];	temp[3]=obs_mf[2][r];	temp[4]=obs_mf[1][f];	temp[5]=obs_mf[3][b];
   mini=minimum(temp);
   if(mini>maximum)
   {
      maximum=mini;
   }
               }
           }
       }
   }
}
 MotorSpeed_mf[0][3]=maximum;
 printf("     Left Motor Stop = %f \n",MotorSpeed_mf[0][3]);
}

// ************** Back Slow **********************
{
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=mini;
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
MotorSpeed_mf[0][2]=maximum;
printf("   Left Motor Back_S = %f \n",MotorSpeed_mf[0][2]);
}
// **************Back Medium ********************
{
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
    MotorSpeed_mf[0][1]=maximum;
    printf("   Left Motor Back_M = %f \n",MotorSpeed_mf[0][1]);
}
// ************* Back Fast  *********************
{
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);

 MotorSpeed_mf[0][0]=maximum;
 printf("   Left Motor Back_F = %f \n",MotorSpeed_mf[0][0]);
}

// ***************** Right Motor ****************
// ****************Forward Fast  ****************
{
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);	
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
	
MotorSpeed_mf[1][6]=maximum;
printf("*********** Right Motor *********** \n");
printf("Right motor Forward_F = %f \n",MotorSpeed_mf[1][6]);
}
//************* Forward Medium ******************
{
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
MotorSpeed_mf[1][5]=maximum;
printf("Right motor Forward_M = %f \n",MotorSpeed_mf[1][5]);
}
// **************Forward Slow ********************
{
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
    maximum=maxi(mini,maximum);
	
MotorSpeed_mf[1][4]=maximum;
printf("Right motor Forward_S = %f \n",MotorSpeed_mf[1][4]);
}
//**************  Stop ***************************
{
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);

for(k=0;k<4;k++)
{
   for(l=0;l<3;l++)
   {
       for(r=0;r<3;r++)
       {
           for(f=0;f<3;f++)
           {
               for(b=0;b<3;b++)
               {
   temp[0]=T_dis_mf[0];	temp[1]=T_dir_mf[k];	temp[2]=obs_mf[0][l];	temp[3]=obs_mf[2][r];	temp[4]=obs_mf[1][f];	temp[5]=obs_mf[3][b];
   mini=minimum(temp);
   if(mini>maximum)
   {
      maximum=mini;
   }
               }
           }
       }
   }
}
 MotorSpeed_mf[1][3]=maximum;
 printf("     Right Motor Stop = %f \n",MotorSpeed_mf[1][3]);
}
// *************** Backward Slow ****************
{
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
MotorSpeed_mf[1][2]=maximum;
printf("   Right motor Back_S = %f \n",MotorSpeed_mf[1][2]);
}
// *****************Backward Medium *************
{
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[0];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[1];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[3];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][1];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][2];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][0];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][2];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[1];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
MotorSpeed_mf[1][1]=maximum;
printf("   Right motor Back_M = %f \n",MotorSpeed_mf[1][1]);
}
// ***************** Backward Fast  *************
{
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][1];	temp[3]=obs_mf[2][1];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=mini;
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][2];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][1];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
temp[0]=T_dis_mf[2];	temp[1]=T_dir_mf[2];	temp[2]=obs_mf[0][0];	temp[3]=obs_mf[2][0];	temp[4]=obs_mf[1][0];	temp[5]=obs_mf[3][2];
mini=minimum(temp);
maximum=maxi(mini,maximum);
MotorSpeed_mf[1][0]=maximum;
printf("   Right motor Back_F = %f \n",MotorSpeed_mf[1][0]);
}

// ********************Defuzzification ***********
// Left motor speed
{
    printf("*********** Output ********* \n");
   // int d;
for (d=0;d<7;d++)
{
    bases[0]=(((MS[d+2]-MS[d+1])/(0.0-1.0))*(MotorSpeed_mf[0][d]-1.0)+MS[d+1])-(((MS[d+1]-MS[d])/(1.0-0.0))*(MotorSpeed_mf[0][d]-0.0)+MS[d]);
    bases[1]=MS[d+2]-MS[d];
    Areas[0][d]=0.5*MotorSpeed_mf[0][d]*(bases[0]+bases[1]);
   // printf("%f %f %f %f %i \n",bases[0],bases[1],Areas[0][d],MotorSpeed_mf[0][d],d);
}
L_MotorSpeed=((MS[1]*Areas[0][0])+(MS[2]*Areas[0][1])+(MS[3]*Areas[0][2])+(MS[4]*Areas[0][3])+(MS[5]*Areas[0][4])+(MS[6]*Areas[0][5])+(MS[7]*Areas[0][6]))/(Areas[0][0]+Areas[0][1]+Areas[0][2]+Areas[0][3]+Areas[0][4]+Areas[0][5]+Areas[0][6]);
//printf("Areas = {%f,%f,%f,%f,%f,%f,%f} \n",Areas[0][0],Areas[0][1],Areas[0][2],Areas[0][3],Areas[0][4],Areas[0][5],Areas[0][6]);
printf("Left Motor Speed = %f \n",L_MotorSpeed);
}
// Right motor speed
{
   // int d;
for (d=0;d<7;d++)
{
    bases[0]=(((MS[d+2]-MS[d+1])/(0.0-1.0))*(MotorSpeed_mf[1][d]-1.0)+MS[d+1])-(((MS[d+1]-MS[d])/(1.0-0.0))*(MotorSpeed_mf[1][d]-0.0)+MS[d]);
    bases[1]=MS[d+2]-MS[d];
    Areas[1][d]=0.5*MotorSpeed_mf[1][d]*(bases[0]+bases[1]);
    //printf("%f %f %f %f %i \n",bases[0],bases[1],Areas[1][d],MotorSpeed_mf[1][d],d);
}
R_MotorSpeed=((MS[1]*Areas[1][0])+(MS[2]*Areas[1][1])+(MS[3]*Areas[1][2])+(MS[4]*Areas[1][3])+(MS[5]*Areas[1][4])+(MS[6]*Areas[1][5])+(MS[7]*Areas[1][6]))/(Areas[1][0]+Areas[1][1]+Areas[1][2]+Areas[1][3]+Areas[1][4]+Areas[1][5]+Areas[1][6]);
//printf("Areas = {%f,%f,%f,%f,%f,%f,%f} \n",Areas[1][0],Areas[1][1],Areas[1][2],Areas[1][3],Areas[1][4],Areas[1][5],Areas[1][6]);
printf("Right Motor Speed = %f \n",R_MotorSpeed);
}
y[0]=L_MotorSpeed;
y[1]=R_MotorSpeed;
}
#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/
 float maxi(float min_val,float max_val)
{
    if(min_val>max_val)
    {
        return min_val;
    }
    else
    {
        return max_val;
    }
}

//
float minimum(float num[6])
{
    float minn=num[0];
    int i;
    for(i=1;i<6;i++)
    {
        if(num[i]<minn)
        {
            minn=num[i];
        }
    }
    return minn;
}
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
