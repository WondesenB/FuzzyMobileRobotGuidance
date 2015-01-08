
#define S_FUNCTION_NAME  Dis_Dir
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include "math.h"
#include "mex.h"
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
int count=1;
float last_lat;
float last_lon;
float temp_lat;
float temp_lon;
float  Distance(float lat1,float lon1,float lat2,float lon2);
float Direction(float lat1,float lon1,float lat2,float lon2);
float DegToRad(float deg);
float RadToDeg(float deg);

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

    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 2);
    ssSetInputPortWidth(S, 1, 2);
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
     ssSetInputPortRequiredContiguous(S, 1, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);

    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetOutputPortWidth(S, 0, 2);
    ssSetOutputPortWidth(S, 1, 1);

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
    const real_T *u0 = (const real_T*) ssGetInputPortSignal(S,0);
    const real_T *u1 = (const real_T*) ssGetInputPortSignal(S,1);
    real_T       *y0 = (real_T*)ssGetOutputPortSignal(S,0);
    real_T       *y1 = (real_T*)ssGetOutputPortSignal(S,1);
    
  
    float theta1;
    float theta2;
    float theta;
    real_T t;
    t=(ssGetT(S));
    if(fmod(t,3)==2.0)
    {
    temp_lat=u0[0];
    temp_lon=u0[1];  
    }
    if(fmod(t,3)==0 || t<0.2)
    {
    last_lat=u0[0];
    last_lon=u0[1];
     
    }
    y0[0]=Distance(u0[0],u0[1],u1[0],u1[1]);
    if((fmod(t,3)>=2.90 ||fmod(t,3)<=0.10 ) && t>0.5)
    {
        theta1=Direction(temp_lat,temp_lon,u0[0],u0[1]);
    }
    else
    {
        theta1=Direction(last_lat,last_lon,u0[0],u0[1]);
    }
    theta2=Direction(u0[0],u0[1],u1[0],u1[1]);
    
    theta=(theta1-theta2);
    if(y0[0]==0.00)
    {
        theta=0.0;
    }
    if(theta<0)
    {
        y0[1]=theta+360;
    }
    else
    {
       y0[1]=theta; 
    }
    y1[0]=theta1;
    printf("Sim time =  ");
    printf("%f % \n",t);
    printf("modulo =  ");
    printf("%f % \n",fmod(t,5));
    printf("Direction to target =  ");
    printf("%f % \n",y0[1]);
    printf("Distance to target =  ");
    printf("%f % \n",y0[0]);
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

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

float Distance(float lat1,float lon1,float lat2,float lon2)
{
  float d = acos(sin(DegToRad(lat1))*sin(DegToRad(lat2))+cos(DegToRad(lat1))*cos(DegToRad(lat2))*cos(DegToRad(lon2-lon1)))*6371000;
  return d;
}
float Direction(float lat1,float lon1,float lat2,float lon2)
{
    float dlon=DegToRad(lon2-lon1);
    float y=sin(dlon)*cos(DegToRad(lat2));
    float x=cos(DegToRad(lat1))*sin(DegToRad(lat2))-sin(DegToRad(lat1))*cos(DegToRad(lat2))*cos(dlon);
    float brng=RadToDeg(atan2(y,x));
    if (brng >= 0)
    {
        return brng;
    }
    else
    {
        return (brng+360);
    }
    
}
float DegToRad(float deg)
{
 return ((3.141592653589793/180)) * deg;
}
float RadToDeg(float deg)
{
 return ((180/3.141592653589793)) * deg;
}