/* jSSC (Java Simple Serial Connector) - serial port communication library.
 * © Alexey Sokolov (scream3r), 2010-2014.
 *
 * This file is part of jSSC.
 *
 * jSSC is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jSSC is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with jSSC.  If not, see <http://www.gnu.org/licenses/>.
 *
 * If you use jSSC in public project you can inform me about this by e-mail,
 * of course if you want it.
 *
 * e-mail: scream3r.org@gmail.com
 * web-site: http://scream3r.org | http://code.google.com/p/java-simple-serial-connector/
 */

#include <fcntl.h>
#include <termios.h>
#include <errno.h>//-D_TS_ERRNO use for Solaris C++ compiler

#include "serial.h"
#include <jni.h>
#include "../jssc_SerialNativeInterface.h"

//#include <iostream> //-lCstd use for Solaris linker

/*
 * Get native library version
 */
JNIEXPORT jstring JNICALL Java_jssc_SerialNativeInterface_getNativeLibraryVersion(JNIEnv *env, jobject object) {
    return env->NewStringUTF(jSSC_NATIVE_LIB_VERSION);
}

/* OK */
/*
 * Port opening
 * 
 * In 2.2.0 added useTIOCEXCL
 */
JNIEXPORT jlong JNICALL Java_jssc_SerialNativeInterface_openPort(JNIEnv *env, jobject object, jstring portName, jboolean useTIOCEXCL){
    const char* port = env->GetStringUTFChars(portName, JNI_FALSE);
    jlong hComm = openPort(port);
    if(hComm != -1){
        if(useTIOCEXCL == JNI_TRUE){
            setExclusiveAccess(hComm, 1);
        }
        
        // TODO: Would like to use non-blocking mode, so keep NDELAY set.
        //since 2.2.0 -> (check termios structure for separating real serial devices from others)
        termios settings;
        if(tcgetattr(hComm, &settings) == 0){
            int flags = fcntl(hComm, F_GETFL, 0);
            flags &= ~O_NDELAY;
            fcntl(hComm, F_SETFL, flags);
        }
        else {
            closePort(hComm);//since 2.7.0
            hComm = jssc_SerialNativeInterface_ERR_INCORRECT_SERIAL_PORT;//-4;
        }
        //<- since 2.2.0
    }
    else {//since 0.9 ->
        if(errno == EBUSY){//Port busy
            hComm = jssc_SerialNativeInterface_ERR_PORT_BUSY;//-1
        }
        else if(errno == ENOENT){//Port not found
            hComm = jssc_SerialNativeInterface_ERR_PORT_NOT_FOUND;//-2;
        }//-> since 2.2.0
        else if(errno == EACCES){//Permission denied
            hComm = jssc_SerialNativeInterface_ERR_PERMISSION_DENIED;//-3;
        }
        else {
            hComm = jssc_SerialNativeInterface_ERR_PORT_NOT_FOUND;//-2;
        }//<- since 2.2.0
    }//<- since 0.9
    env->ReleaseStringUTFChars(portName, port);
    return hComm;
}

/* OK */
/*
 * Set serial port settings
 *
 * In 2.6.0 added flags parameter
 */
JNIEXPORT jboolean JNICALL Java_jssc_SerialNativeInterface_setParams
  (JNIEnv *env, jobject object, jlong portHandle, jint baudRate, jint byteSize, jint stopBits, jint parity, jboolean setRTS, jboolean setDTR, jint flags){
    return setParams(portHandle, baudRate, byteSize, stopBits, parity, setRTS ? 1 : 0, setDTR ? 1 : 0, flags) == 0;
}


/* OK */
/*
 * PurgeComm
 */
JNIEXPORT jboolean JNICALL Java_jssc_SerialNativeInterface_purgePort
  (JNIEnv *env, jobject object, jlong portHandle, jint flags){
    return purgePort(portHandle, flags)  == 0 ? JNI_TRUE : JNI_FALSE;
}

/* OK */
/* Closing the port */
JNIEXPORT jboolean JNICALL Java_jssc_SerialNativeInterface_closePort
  (JNIEnv *env, jobject object, jlong portHandle){
    setExclusiveAccess(portHandle, 0);
    return closePort(portHandle) == 0 ? JNI_TRUE : JNI_FALSE;
}

/* OK */
/*
 * Setting events mask
 */
JNIEXPORT jboolean JNICALL Java_jssc_SerialNativeInterface_setEventsMask
  (JNIEnv *env, jobject object, jlong portHandle, jint mask){
    //Don't needed in linux, implemented in java code
    return JNI_TRUE;
}

/* OK */
/*
 * Getting events mask
 */
JNIEXPORT jint JNICALL Java_jssc_SerialNativeInterface_getEventsMask
  (JNIEnv *env, jobject object, jlong portHandle){
    //Don't needed in linux, implemented in java code
    return -1;
}

/* OK */
/* 
 * RTS line status changing (ON || OFF)
 */
JNIEXPORT jboolean JNICALL Java_jssc_SerialNativeInterface_setRTS
  (JNIEnv *env, jobject object, jlong portHandle, jboolean enabled){
    return setRTS(portHandle, enabled ? 1 : 0) == 0 ? JNI_TRUE : JNI_FALSE;
}

/* OK */
/* 
 * DTR line status changing (ON || OFF)
 */
JNIEXPORT jboolean JNICALL Java_jssc_SerialNativeInterface_setDTR
  (JNIEnv *env, jobject object, jlong portHandle, jboolean enabled){
    return setDTR(portHandle, enabled ? 1 : 0) == 0 ? JNI_TRUE : JNI_FALSE;
}

/* OK */
/*
 * Writing data to the port
 * TODO: This seems buggy for me.
 * The syscall write does not guarantee / need to write everything in one run.
 */
JNIEXPORT jboolean JNICALL Java_jssc_SerialNativeInterface_writeBytes
  (JNIEnv *env, jobject object, jlong portHandle, jbyteArray buffer){
    jbyte* jBuffer = env->GetByteArrayElements(buffer, JNI_FALSE);
    jint bufferSize = env->GetArrayLength(buffer);
    jint result = writeBytes(portHandle, jBuffer, (size_t)bufferSize);
    env->ReleaseByteArrayElements(buffer, jBuffer, 0);
    return result == bufferSize ? JNI_TRUE : JNI_FALSE;
}

/* OK */
/*
 * Reading data from the port
 *
 * Rewrited in 2.5.0 (using select() function for correct block reading in MacOS X)
 */
JNIEXPORT jbyteArray JNICALL Java_jssc_SerialNativeInterface_readBytes
  (JNIEnv *env, jobject object, jlong portHandle, jint byteCount){
    jbyte *lpBuffer = new jbyte[byteCount];
    const jint readCnt = readBytes(portHandle, lpBuffer, byteCount);
    jbyteArray returnArray = env->NewByteArray(readCnt);
    env->SetByteArrayRegion(returnArray, 0, readCnt, lpBuffer);
    delete lpBuffer;
    return returnArray;
}

/* OK */
/*
 * Get bytes count in serial port buffers (Input and Output)
 */
JNIEXPORT jintArray JNICALL Java_jssc_SerialNativeInterface_getBuffersBytesCount
  (JNIEnv *env, jobject object, jlong portHandle){
    jint returnValues[2];
    returnValues[0] = getBufferBytesCountIn(portHandle); //Input buffer
    returnValues[1] = getBufferBytesCountOut(portHandle); //Output buffer
    jintArray returnArray = env->NewIntArray(2);
    env->SetIntArrayRegion(returnArray, 0, 2, returnValues);
    return returnArray;
}


/* OK */
/*
 * Setting flow control mode
 */
JNIEXPORT jboolean JNICALL Java_jssc_SerialNativeInterface_setFlowControlMode
  (JNIEnv *env, jobject object, jlong portHandle, jint mask) {
    return setFlowControlMode(portHandle, mask) == 0 ? JNI_TRUE : JNI_FALSE;
}

/* OK */
/*
 * Getting flow control mode
 */
JNIEXPORT jint JNICALL Java_jssc_SerialNativeInterface_getFlowControlMode
  (JNIEnv *env, jobject object, jlong portHandle){
    jint mask;
    if (getFlowControlMode(portHandle, &mask) == 0) {
        return mask;
    } else {
        return 0;
    }
}

/* OK */
/*
 * Send break for setted duration
 */
JNIEXPORT jboolean JNICALL Java_jssc_SerialNativeInterface_sendBreak
  (JNIEnv *env, jobject object, jlong portHandle, jint duration){
    return sendBreak(portHandle, duration) == 0 ? JNI_TRUE : JNI_FALSE;
}

const jint INTERRUPT_BREAK = 512;
const jint INTERRUPT_TX = 1024;
const jint INTERRUPT_FRAME = 2048;
const jint INTERRUPT_OVERRUN = 4096;
const jint INTERRUPT_PARITY = 8192;

const jint EV_CTS = 8;
const jint EV_DSR = 16;
const jint EV_RING = 256;
const jint EV_RLSD = 32;
const jint EV_RXCHAR = 1;
//const jint EV_RXFLAG = 2; //Not supported
const jint EV_TXEMPTY = 4;
const jint events[] = {INTERRUPT_BREAK,
                       INTERRUPT_TX,
                       INTERRUPT_FRAME,
                       INTERRUPT_OVERRUN,
                       INTERRUPT_PARITY,
                       EV_CTS,
                       EV_DSR,
                       EV_RING,
                       EV_RLSD,
                       EV_RXCHAR,
                       //EV_RXFLAG, //Not supported
                       EV_TXEMPTY};

void helperWaitEventsAdd(JNIEnv *env, jobjectArray* returnArray, int* i, int key, int value) {
    jint returnValues[] = { key, value };
    jintArray singleResultArray = env->NewIntArray(2);
    env->SetIntArrayRegion(singleResultArray, 0, 2, returnValues);
    env->SetObjectArrayElement(*returnArray, *i, singleResultArray);
    ++*i;
}

/* OK */
/*
 * Collecting data for EventListener class (Linux have no implementation of "WaitCommEvent" function from Windows)
 * 
 */
JNIEXPORT jobjectArray JNICALL Java_jssc_SerialNativeInterface_waitEvents
  (JNIEnv *env, jobject object, jlong portHandle) {
    jclass intClass = env->FindClass("[I");
    jobjectArray returnArray = env->NewObjectArray(sizeof(events)/sizeof(jint), intClass, NULL);
    int i = 0;

    struct interrupt_t it;
    struct line_status_t l;

    if (getInterruptsCount(portHandle, &it) == 0) {
        helperWaitEventsAdd(env, &returnArray, &i, INTERRUPT_BREAK, it.i_break);
        helperWaitEventsAdd(env, &returnArray, &i, INTERRUPT_TX, it.i_tx);
        helperWaitEventsAdd(env, &returnArray, &i, INTERRUPT_FRAME, it.i_frame);
        helperWaitEventsAdd(env, &returnArray, &i, INTERRUPT_OVERRUN, it.i_overrun);
        helperWaitEventsAdd(env, &returnArray, &i, INTERRUPT_PARITY, it.i_parity);
    }

    if (getLinesStatus(portHandle, &l) == 0) {
        helperWaitEventsAdd(env, &returnArray, &i, EV_CTS, l.l_cts);
        helperWaitEventsAdd(env, &returnArray, &i, EV_DSR, l.l_dsr);
        helperWaitEventsAdd(env, &returnArray, &i, EV_RING, l.l_ring);
        helperWaitEventsAdd(env, &returnArray, &i, EV_RLSD, l.l_rlsd);
    }
    
    helperWaitEventsAdd(env, &returnArray, &i, EV_RXCHAR, getBufferBytesCountIn(portHandle));
    //helperWaitEventsAdd(env, &returnArray, &i, EV_RXFLAG, 0);
    helperWaitEventsAdd(env, &returnArray, &i, EV_TXEMPTY, getBufferBytesCountOut(portHandle));

    return returnArray;
}

/* OK */
/*
 * Getting serial ports names like an a String array (String[])
 */
JNIEXPORT jobjectArray JNICALL Java_jssc_SerialNativeInterface_getSerialPortNames
  (JNIEnv *env, jobject object){
    //Don't needed in linux, implemented in java code (Note: null will be returned)
    return NULL;
}

/* OK */
/*
 * Getting lines status
 *
 * returnValues[0] - CTS
 * returnValues[1] - DSR
 * returnValues[2] - RING
 * returnValues[3] - RLSD(DCD)
 */
JNIEXPORT jintArray JNICALL Java_jssc_SerialNativeInterface_getLinesStatus
  (JNIEnv *env, jobject object, jlong portHandle){
    jint returnValues[4] = {0};
    
    jintArray returnArray = env->NewIntArray(4);

    /*Lines status*/
    struct line_status_t l;
    if (getLinesStatus(portHandle, &l) == 0) {
        returnValues[0] = l.l_cts; /*CTS status*/
        returnValues[1] = l.l_dsr; /*DSR status*/
        returnValues[2] = l.l_ring; /*RING status*/
        returnValues[3] = l.l_rlsd; /*RLSD(DCD) status*/
    }

    env->SetIntArrayRegion(returnArray, 0, 4, returnValues);
    return returnArray;
}
