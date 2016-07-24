/* ***********************************************************************************************

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2013-2015 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  BSD LICENSE

  Copyright(c) 2013-2015 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  ***********************************************************************************************
*/

#ifndef __ERROR_REPORTING_UTILS_H__
#define __ERROR_REPORTING_UTILS_H__

#define DRV_ASSERT_N_RET_VAL(ret_val)                                    \
    DRV_ASSERT((ret_val) == VT_SUCCESS);                                 \
    DRV_CHECK_N_RETURN_N_FAIL(ret_val);

#define DRV_ASSERT_N_CONTINUE(ret_val)                                   \
    if ((ret_val) != VT_SUCCESS) {                                       \
        LOG_ERR1(VTSA_T("Operation failed with error code "),(ret_val)); \
    }

#define DRV_CHECK_N_RETURN_N_FAIL(ret_val)                               \
    if ((ret_val) != VT_SUCCESS) {                                       \
        LOG_ERR1(VTSA_T("Operation failed with error code "),(ret_val)); \
        return (ret_val);                                                \
    }

#define DRV_CHECK_N_RETURN_NO_RETVAL(ret_val)                            \
    if ((ret_val) != VT_SUCCESS) {                                       \
        LOG_ERR1(VTSA_T("Operation failed with error code "),(ret_val)); \
        return;                                                          \
    }

#define DRV_CHECK_PTR_N_RET_VAL(ptr)                                     \
    if ((ptr) == NULL) {                                                 \
        LOG_ERR0(VTSA_T("Encountered null pointer"));                    \
        return VT_SAM_ERROR;                                             \
    }

#define DRV_CHECK_PTR_N_RET_NULL(ptr)                                    \
    if ((ptr) == NULL) {                                                 \
        LOG_ERR0(VTSA_T("Encountered null pointer"));                    \
        return NULL;                                                     \
    }

#define DRV_CHECK_PTR_N_LOG_NO_RETURN(ptr)                               \
    if ((ptr) == NULL) {                                                 \
        LOG_ERR0(VTSA_T("Encountered null pointer"));                    \
    }

#define DRV_CHECK_N_LOG_NO_RETURN(ret_val)                               \
    if ((ret_val) != VT_SUCCESS) {                                       \
        LOG_ERR1(VTSA_T("Operation failed with error code "),(ret_val)); \
    }
    
#define DRV_CHECK_N_RET_NEG_ONE(ret_val)                                 \
    if ((ret_val) == -1) {                                               \
        LOG_ERR0(VTSA_T("Operation failed with error code = -1"));       \
        return VT_SAM_ERROR;                                             \
    }

#define DRV_REQUIRES_TRUE_COND_RET_N_FAIL( cond )                        \
    if ( !(cond) ) {                                                     \
        LOG_ERR0(VTSA_T("Condition check failed"));                      \
        return VT_SAM_ERROR;                                             \
    }

#define DRV_REQUIRES_TRUE_COND_RET_ASSIGNED_VAL( cond, ret_val)         \
    if ( !(cond) ) {                                                    \
        LOG_ERR0(VTSA_T("Condition check failed"));                     \
        return ret_val;                                                 \
    }

#define DRV_CHECK_N_ERR_LOG_ERR_STRNG_N_RET( rise_err )                \
    if (rise_err != VT_SUCCESS) {                                      \
        PVOID rise_ptr = NULL;                                         \
        const VTSA_CHAR *error_str = NULL;                             \
        RISE_open(&rise_ptr);                                          \
        RISE_translate_err_code(rise_ptr, rise_err, &error_str);       \
        LogItW(LOG_LEVEL_ERROR|LOG_AREA_GENERAL, L"Operation failed with error [ %d ] = %s\n",rise_err,error_str); \
        RISE_close(rise_ptr);                                          \
        return rise_err;                                               \
    }

#define DRV_CHECK_PTR_N_CLEANUP(ptr, gotolabel, ret_val)                 \
    if ((ptr) == NULL) {                                                 \
        LOG_ERR0(VTSA_T("Encountered null pointer"));                    \
        ret_val = VT_SAM_ERROR;                                          \
        goto gotolabel;                                                  \
    }

#define DRV_CHECK_ON_FAIL_CLEANUP_N_RETURN(ret_val, gotolabel)         \
    if ((ret_val) != VT_SUCCESS) {                                     \
        DRV_CHECK_N_LOG_NO_RETURN(ret_val);                            \
        goto gotolabel;                                                \
    } 


#define DRV_CHECK_N_CLEANUP_N_RETURN_RET_NEG_ONE(ret_val, gotolabel)   \
    if ((ret_val) == -1) {                                             \
        DRV_CHECK_N_LOG_NO_RETURN(ret_val);                            \
        goto gotolabel;                                                \
    } 

#define DRV_CHECK_PTR_ON_NULL_CLEANUP_N_RETURN(ptr, gotolabel)         \
    if ((ptr) == NULL) {                                               \
        DRV_CHECK_PTR_N_LOG_NO_RETURN(ptr);                            \
        goto gotolabel;                                                \
    } 
    
#define FREE_N_SET_NULL(ptr)                                           \
    if (ptr != NULL) {                                                 \
        free(ptr);                                                     \
        ptr = NULL;                                                    \
    }

#define DELETE_N_SET_NULL(ptr)                                         \
        delete ptr;                                                    \
        ptr = NULL;

#endif

