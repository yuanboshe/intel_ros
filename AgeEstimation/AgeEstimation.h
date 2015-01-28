//
// MATLAB Compiler: 4.18.1 (R2013a)
// Date: Wed Jan 07 13:30:20 2015
// Arguments: "-B" "macro_default" "-W" "cpplib:AgeEstimation" "-T" "link:lib"
// "-d" "E:\code\Matlab\FaceBiometrics\AgeEstimation_32\AgeEstimation\src" "-w"
// "enable:specified_file_mismatch" "-w" "enable:repeated_file" "-w"
// "enable:switch_ignored" "-w" "enable:missing_lib_sentinel" "-w"
// "enable:demo_license" "-v"
// "E:\code\Matlab\FaceBiometrics\AgeEstimation_32\estimate_age_matlab.m"
// "E:\code\Matlab\FaceBiometrics\AgeEstimation_32\load_aging_model_matlab.m" 
//

#ifndef __AgeEstimation_h
#define __AgeEstimation_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SUNPRO_CC)
/* Solaris shared libraries use __global, rather than mapfiles
 * to define the API exported from a shared library. __global is
 * only necessary when building the library -- files including
 * this header file to use the library do not need the __global
 * declaration; hence the EXPORTING_<library> logic.
 */

#ifdef EXPORTING_AgeEstimation
#define PUBLIC_AgeEstimation_C_API __global
#else
#define PUBLIC_AgeEstimation_C_API /* No import statement needed. */
#endif

#define LIB_AgeEstimation_C_API PUBLIC_AgeEstimation_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_AgeEstimation
#define PUBLIC_AgeEstimation_C_API __declspec(dllexport)
#else
#define PUBLIC_AgeEstimation_C_API __declspec(dllimport)
#endif

#define LIB_AgeEstimation_C_API PUBLIC_AgeEstimation_C_API


#else

#define LIB_AgeEstimation_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_AgeEstimation_C_API 
#define LIB_AgeEstimation_C_API /* No special import/export declaration */
#endif

extern LIB_AgeEstimation_C_API 
bool MW_CALL_CONV AgeEstimationInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_AgeEstimation_C_API 
bool MW_CALL_CONV AgeEstimationInitialize(void);

extern LIB_AgeEstimation_C_API 
void MW_CALL_CONV AgeEstimationTerminate(void);



extern LIB_AgeEstimation_C_API 
void MW_CALL_CONV AgeEstimationPrintStackTrace(void);

extern LIB_AgeEstimation_C_API 
bool MW_CALL_CONV mlxEstimate_age_matlab(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                         *prhs[]);

extern LIB_AgeEstimation_C_API 
bool MW_CALL_CONV mlxLoad_aging_model_matlab(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                             *prhs[]);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_AgeEstimation
#define PUBLIC_AgeEstimation_CPP_API __declspec(dllexport)
#else
#define PUBLIC_AgeEstimation_CPP_API __declspec(dllimport)
#endif

#define LIB_AgeEstimation_CPP_API PUBLIC_AgeEstimation_CPP_API

#else

#if !defined(LIB_AgeEstimation_CPP_API)
#if defined(LIB_AgeEstimation_C_API)
#define LIB_AgeEstimation_CPP_API LIB_AgeEstimation_C_API
#else
#define LIB_AgeEstimation_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_AgeEstimation_CPP_API void MW_CALL_CONV estimate_age_matlab(int nargout, mwArray& age, const mwArray& im, const mwArray& w, const mwArray& knn_train_x, const mwArray& knn_train_y);

extern LIB_AgeEstimation_CPP_API void MW_CALL_CONV load_aging_model_matlab(int nargout, mwArray& w, mwArray& trainx, mwArray& trainy, const mwArray& filename);

#endif
#endif
