/*
 * ADEuresys.cpp
 *
 * This is a driver for Euresys frame grabbers
 *
 * Author: Mark Rivers
 *         University of Chicago
 *
 * Contributor: Jan Jug
 *              Cosylab d.d.
 *              jan.jug@cosylab.com
 *
 * Created: March 6, 2024
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <cantProceed.h>
#include <epicsString.h>
#include <epicsExit.h>

#include <ADGenICam.h>
#include <EGrabber.h>
using namespace Euresys;

#include <epicsExport.h>
#include "ADEuresys.h"
#include "EuresysFeature.h"

#define DRIVER_VERSION      1
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

static const char *driverName = "ADEuresys";

typedef enum {
    TimeStampCamera,
    TimeStampEPICS
} ESTimeStamp_t;

typedef enum {
    UniqueIdCamera,
    UniqueIdDriver
} ESUniqueId_t;

/** Original configuration function for backward compatibility.
 *
 * This function maintains backward compatibility with the old API. The cameraId parameter
 * is ignored (it was never used in the original implementation). This always connects to
 * interface 0, device 0, which matches the original behavior.
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] cameraId Legacy parameter (ignored - kept for backward compatibility).
 * \param[in] numEGBuffers The number of buffers to allocate in EGrabber.
 *            If set to 0 or omitted the default of 100 will be used.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
extern "C" int ADEuresysConfig(const char *portName, const char* cameraId,
                               int numEGBuffers, size_t maxMemory, int priority, int stackSize)
{
    new ADEuresys(portName, cameraId, numEGBuffers, maxMemory, priority, stackSize);
    return asynSuccess;
}

/** Configuration function to configure one camera with explicit interface and device indices.
 *
 * This function need to be called once for each camera to be used by the IOC. A call to this
 * function instantiates one object from the ADEuresys class.
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] interfaceIndex The GenTL interface index (typically the board/card index).
 * \param[in] deviceIndex The GenTL device index (camera index on the specified interface).
 * \param[in] numEGBuffers The number of buffers to allocate in EGrabber.
 *            If set to 0 or omitted the default of 100 will be used.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
extern "C" int ADEuresysConfig2(const char *portName, int interfaceIndex, int deviceIndex,
                               int numEGBuffers, size_t maxMemory, int priority, int stackSize)
{
    new ADEuresys(portName, interfaceIndex, deviceIndex, numEGBuffers,
                  maxMemory, priority, stackSize);
    return asynSuccess;
}

class myGrabber : public EGRABBER_CALLBACK {
public:
    myGrabber(EGenTL *gentl, int interfaceIndex, int deviceIndex,
              class ADEuresys *pADEuresys)
        : EGRABBER_CALLBACK(*gentl, interfaceIndex, deviceIndex) {
        pADEuresys_ = pADEuresys;
        enableEvent<NewBufferData>();
    }
private:
    class ADEuresys *pADEuresys_;
    virtual void onNewBufferEvent(const NewBufferData &data) {
        ScopedBuffer buf(*this, data);
        pADEuresys_->processFrame(buf);
    }
};


static void c_shutdown(void *arg)
{
   ADEuresys *p = (ADEuresys *)arg;
   p->shutdown();
}


/** Legacy constructor for backward compatibility.
 * Ignores cameraId and uses interface 0, device 0 (matching original behavior).
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] cameraId Legacy parameter (ignored).
 * \param[in] numEGBuffers The number of buffers to allocate in EGrabber.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
ADEuresys::ADEuresys(const char *portName, const char* cameraId, int numEGBuffers,
                     size_t maxMemory, int priority, int stackSize)
    : ADEuresys(portName, 0, 0, numEGBuffers, maxMemory, priority, stackSize)
{
    // Delegate to new constructor with interface 0, device 0
    // Note: cameraId parameter is ignored as it was never used in the original implementation
}

/** Constructor for the ADEuresys class with explicit interface and device indices.
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] interfaceIndex The GenTL interface index (typically the board/card index).
 * \param[in] deviceIndex The GenTL device index (camera index on the specified interface).
 * \param[in] numEGBuffers The number of buffers to allocate in EGrabber.
 *            If set to 0 or omitted the default of 100 will be used.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
ADEuresys::ADEuresys(const char *portName, int interfaceIndex, int deviceIndex,
                     int numEGBuffers, size_t maxMemory, int priority, int stackSize )
    : ADGenICam(portName, maxMemory, priority, stackSize),
    numEGBuffers_(numEGBuffers), exiting_(0), uniqueId_(0)
{
    static const char *functionName = "ADEuresys";

    //pasynTrace->setTraceMask(pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACE_WARNING | ASYN_TRACEIO_DRIVER);

    if (numEGBuffers_ == 0) numEGBuffers_ = 100;

    /* GenTL system management singleton via shared/weak pointer */
    static std::weak_ptr<EGenTL> staticGenTL;

    // init genTL
    pGenTL_ = staticGenTL.lock();               // get the shared pointer if it exists
    if (!pGenTL_) {                             // if it does not exist, create it
        pGenTL_ = std::make_shared<EGenTL>();
        staticGenTL = pGenTL_;
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s: created GenTL system singleton\n",
                    driverName, functionName);
    }

    try {
        // Create EGrabber with specific interface and device indices using singleton
        mGrabber_ = new myGrabber(pGenTL_.get(), interfaceIndex, deviceIndex, this);
        mGrabber_->reallocBuffers(numEGBuffers);

        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s::%s connected to interface %d, device %d\n",
                  driverName, functionName, interfaceIndex, deviceIndex);
    }
    catch (std::exception &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s ERROR creating EGrabber for interface %d, device %d: %s\n",
                  driverName, functionName, interfaceIndex, deviceIndex, e.what());
        // Clean up mGrabber_ if it was created but reallocBuffers failed
        if (mGrabber_) {
            delete mGrabber_;
            mGrabber_ = NULL;
        }
        throw;
    }
 
    createParam(ESTimeStampModeString,              asynParamInt32,   &ESTimeStampMode);
    createParam(ESUniqueIdModeString,               asynParamInt32,   &ESUniqueIdMode);
    createParam(ESBufferSizeString,                 asynParamInt32,   &ESBufferSize);
    createParam(ESOutputQueueString,                asynParamInt32,   &ESOutputQueue);
    createParam(ESRejectedFramesString,             asynParamInt32,   &ESRejectedFrames);
    createParam(ESCRCErrorCountString,             asynParamInt32,    &ESCRCErrorCount);
    createParam(ESResetErrorCountsString,           asynParamInt32,   &ESResetErrorCounts);
    createParam(ESProcessTotalTimeString,         asynParamFloat64,   &ESProcessTotalTime);
    createParam(ESProcessCopyTimeString,          asynParamFloat64,   &ESProcessCopyTime);
    createParam(ESConvertPixelFormatString,         asynParamInt32,   &ESConvertPixelFormat);
    createParam(ESUnpackingModeString,              asynParamInt32,   &ESUnpackingMode);

    /* Set initial values of some parameters */
    setIntegerParam(ESBufferSize, numEGBuffers);
    setIntegerParam(ESOutputQueue, 0);
    setIntegerParam(NDDataType, NDUInt8);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setIntegerParam(ADMinX, 0);
    setIntegerParam(ADMinY, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");
    char driverVersionString[256];
    epicsSnprintf(driverVersionString, sizeof(driverVersionString), "%d.%d.%d", 
                  DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
    setStringParam(NDDriverVersion,driverVersionString);
    setStringParam(ADSDKVersion, Euresys::Internal::EGrabberClientVersion);
    resetErrorCounts();
    
    // shutdown on exit
    epicsAtExit(c_shutdown, this);
    return;
}

EGRABBER_CALLBACK* ADEuresys::getGrabber() {
    return mGrabber_;
}

void ADEuresys::shutdown(void)
{
    lock();
    exiting_ = 1;
    stopCapture();
    if (mGrabber_) {
        delete mGrabber_;
        mGrabber_ = NULL;
    }
    unlock();

    pGenTL_.reset();
}

GenICamFeature *ADEuresys::createFeature(GenICamFeatureSet *set, 
                                         std::string const & asynName, asynParamType asynType, int asynIndex,
                                         std::string const & featureName, GCFeatureType_t featureType) {
    // See if the feature name begins with strings that specify a module
    if (asynName.find("DS_") == 0) {
        return new EuresysFeature<StreamModule>(set, asynName, asynType, asynIndex, featureName, featureType);
    }
    else if (asynName.find("DV_") == 0) {
        return new EuresysFeature<DeviceModule>(set, asynName, asynType, asynIndex, featureName, featureType);
    }
    else if (asynName.find("IF_") == 0) {
        return new EuresysFeature<InterfaceModule>(set, asynName, asynType, asynIndex, featureName, featureType);
    }
    else if (asynName.find("SY_") == 0) {
        return new EuresysFeature<SystemModule>(set, asynName, asynType, asynIndex, featureName, featureType);
    } 
    else {
        return new EuresysFeature<RemoteModule>(set, asynName, asynType, asynIndex, featureName, featureType);
    }                      
}


// This function is called from the EGrabber NewBufferData callback thread
void ADEuresys::processFrame(ScopedBuffer &buf)
{
    NDArray *pRaw = 0;
    size_t nRows, nCols;
    NDDataType_t dataType = NDUInt8;
    NDColorMode_t colorMode = NDColorModeMono;
    int acquire;
    int timeStampMode;
    int uniqueIdMode;
    int numColors=1;
    size_t dims[3];
    int pixelSize;
    size_t dataSize;
    size_t frameSize;
    void *pData;
    int nDims;
    int numImages;
    int imageCounter;
    int numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    epicsUInt64 timeStamp;
    epicsUInt64 frameId;
    BufferInfo bufferInfo;
    epicsTime t1, t2, t3, t4;
    static const char *functionName = "processFrame";

    lock();
    getIntegerParam(ADAcquire, &acquire);
    t1=t2=t3=t4 = epicsTime::getCurrent();
    //If acquisition has stopped then ignore this frame
    if (!acquire) goto done;

    bufferInfo = buf.getInfo();
    
    // Get the image information
    pData = bufferInfo.base;
    nCols = bufferInfo.width;
    nRows = bufferInfo.deliveredHeight;
    pixelSize = ((bufferInfo.bitsPerPixel - 1) / 8) + 1;
    frameSize = bufferInfo.size;
    timeStamp = buf.getInfo<uint64_t>(gc::BUFFER_INFO_TIMESTAMP);
    frameId = buf.getInfo<uint64_t>(gc::BUFFER_INFO_FRAMEID);
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, 
              "%s::%s nCols=%d, nRows=%d, pixelSize=%d, frameSize=%d, pixelFormat=%s, frameId=%llu, timeStamp=%llu\n",
              driverName, functionName, (int)nCols, (int)nRows, pixelSize, (int)frameSize, bufferInfo.pixelFormat.c_str(), frameId, timeStamp);
    switch (pixelSize) {
        case 1:
            dataType = NDUInt8;
            break;
        case 2:
            dataType = NDUInt16;
            break;
        case 4:
            dataType = NDUInt32;
            break;
        default:
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s error unexpected pixelSize=%d\n",
                      driverName, functionName, pixelSize);
    }

    if (numColors == 1) {
        nDims = 2;
        dims[0] = nCols;
        dims[1] = nRows;
    } else {
        nDims = 3;
        dims[0] = 3;
        dims[1] = nCols;
        dims[2] = nRows;
    }
    dataSize = dims[0] * dims[1] * pixelSize;
    if (nDims == 3) dataSize *= dims[2];
    if (dataSize != frameSize) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: data size mismatch: calculated=%lu, reported=%lu\n",
            driverName, functionName, (long)dataSize, (long)frameSize);
    }
    setIntegerParam(NDArraySizeX, (int)nCols);
    setIntegerParam(NDArraySizeY, (int)nRows);
    setIntegerParam(NDArraySize, (int)dataSize);
    setIntegerParam(NDDataType, dataType);
    if (nDims == 3) {
        colorMode = NDColorModeRGB1;
    } 
    setIntegerParam(NDColorMode, colorMode);
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    if (arrayCallbacks) {
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s::%s allocating pRaw\n", driverName, functionName);
        pRaw = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
        if (!pRaw) {
            // If we didn't get a valid buffer from the NDArrayPool we must abort
            // the acquisition as we have nowhere to dump the data...
            setIntegerParam(ADStatus, ADStatusAborting);
            callParamCallbacks();
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s::%s [%s] ERROR: Serious problem: not enough buffers left! Aborting acquisition!\n",
                driverName, functionName, portName);
            setIntegerParam(ADAcquire, 0);
            goto done;
        }
        if (pData) {
            unlock();
            t2 = epicsTime::getCurrent();
            memcpy(pRaw->pData, pData, dataSize);
            t3 = epicsTime::getCurrent();
            lock();
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s::%s [%s] ERROR: pData is NULL!\n",
                driverName, functionName, portName);
            goto done;
        }
        
        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);

        // Put the frame number into the buffer
        getIntegerParam(ESUniqueIdMode, &uniqueIdMode);
        if (uniqueIdMode == UniqueIdCamera) {
            pRaw->uniqueId = (int)frameId;
        } else {
            pRaw->uniqueId = numImagesCounter;
        }
            
        updateTimeStamp(&pRaw->epicsTS);
        getIntegerParam(ESTimeStampMode, &timeStampMode);
        // Set the timestamps in the buffer
        if (timeStampMode == TimeStampCamera) {
            pRaw->timeStamp = (double)timeStamp;
        } else {
            pRaw->timeStamp = pRaw->epicsTS.secPastEpoch + pRaw->epicsTS.nsec/1e9;
        }
    
        // Get any attributes that have been defined for this driver        
        getAttributes(pRaw->pAttributeList);
            
        // Change the status to be readout...
        setIntegerParam(ADStatus, ADStatusReadout);
        
        pRaw->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);
    }

    imageCounter++;
    numImagesCounter++;
    setIntegerParam(NDArrayCounter, imageCounter);
    setIntegerParam(ADNumImagesCounter, numImagesCounter);

    if (arrayCallbacks) {
        // Call the NDArray callback
        doCallbacksGenericPointer(pRaw, NDArrayData, 0);
        // Release the NDArray buffer now that we are done with it.
        if (this->pArrays[0]) {
            this->pArrays[0]->release();
        }
        this->pArrays[0] = pRaw;
        pRaw = NULL;
    }

    // See if acquisition is done if we are in single or multiple mode
    if ((imageMode == ADImageSingle) ||
        ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))) {
        setIntegerParam(ADStatus, ADStatusIdle);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s calling stopCapture\n", driverName, functionName);
        stopCapture();
    }
    t4 = epicsTime::getCurrent();
    setDoubleParam(ESProcessTotalTime, (t4-t1)*1000.);
    setDoubleParam(ESProcessCopyTime, (t3-t2)*1000.);
    
    done:
    callParamCallbacks();
    unlock();
}

asynStatus ADEuresys::readStatus()
{
    mGrabber_->setString<StreamModule>("EventSelector", "RejectedFrame");
    epicsUInt64 rejectedFrames = mGrabber_->getInteger<StreamModule>("EventCount");
    setIntegerParam(ESRejectedFrames, (int)rejectedFrames);
    epicsUInt64 CRCErrors = mGrabber_->getInteger<InterfaceModule>("CxpStreamDataPacketCrcErrorCount");
    setIntegerParam(ESCRCErrorCount, (int)CRCErrors);
    epicsUInt64 outputQueue = mGrabber_->getInfo<StreamModule, uint64_t>(GenTL::STREAM_INFO_NUM_AWAIT_DELIVERY);
    setIntegerParam(ESOutputQueue, (int)outputQueue);
   
    return ADGenICam::readStatus();
}

asynStatus ADEuresys::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    //static const char *functionName = "writeInt32";

    if (function == ESResetErrorCounts) {
        resetErrorCounts();
    } 
    else if (function == ESUnpackingMode) {
        mGrabber_->setInteger<StreamModule>("UnpackingMode", value);
    }
    return ADGenICam::writeInt32(pasynUser, value);
}

void ADEuresys::resetErrorCounts()
{
    //static const char *functionName = "resetErrorCounts";

    mGrabber_->setString<StreamModule>("EventSelector", "RejectedFrame");
    mGrabber_->execute<StreamModule>("EventCountReset");
    mGrabber_->execute<InterfaceModule>("CxpStreamDataPacketCrcErrorCountReset");
}

asynStatus ADEuresys::startCapture()
{
    //static const char *functionName = "startCapture";
    
    int imageMode, numImages;
    getIntegerParam(ADImageMode, &imageMode);
    getIntegerParam(ADNumImages, &numImages);
    if (imageMode == ADImageSingle) numImages = 1;
    if (imageMode == ADImageContinuous) numImages = -1;
    setIntegerParam(ADNumImagesCounter, 0);
    mGrabber_->reallocBuffers(numEGBuffers_);
    mGrabber_->start(numImages);
    return asynSuccess;
}

asynStatus ADEuresys::stopCapture()
{
    //static const char *functionName = "stopCapture";

    mGrabber_->stop();
    setShutter(0);
    setIntegerParam(ADAcquire, 0);
    callParamCallbacks();
    return asynSuccess;
}

/** Print out a report; calls ADGenICam::report to get base class report as well.
  * \param[in] fp File pointer to write output to
  * \param[in] details Level of detail desired.  If >1 prints information about 
               supported video formats and modes, etc.
 */

void ADEuresys::report(FILE *fp, int details)
{
    //static const char *functionName = "report";

    fprintf(fp, "\n");
    fprintf(fp, "Report for camera in use:\n");
    ADGenICam::report(fp, details);
    return;
}

// Legacy iocsh configuration for backward compatibility
static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"Camera ID", iocshArgString};
static const iocshArg configArg2 = {"# EGrabber buffers", iocshArgInt};
static const iocshArg configArg3 = {"maxMemory", iocshArgInt};
static const iocshArg configArg4 = {"priority", iocshArgInt};
static const iocshArg configArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs[] = {&configArg0,
                                                     &configArg1,
                                                     &configArg2,
                                                     &configArg3,
                                                     &configArg4,
                                                     &configArg5};
static const iocshFuncDef configADEuresys = {"ADEuresysConfig", 6, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    ADEuresysConfig(args[0].sval, args[1].sval, args[2].ival,
                    args[3].ival, args[4].ival, args[5].ival);
}

// New iocsh configuration with explicit interface and device indices
static const iocshArg config2Arg0 = {"Port name", iocshArgString};
static const iocshArg config2Arg1 = {"Interface index", iocshArgInt};
static const iocshArg config2Arg2 = {"Device index", iocshArgInt};
static const iocshArg config2Arg3 = {"# EGrabber buffers", iocshArgInt};
static const iocshArg config2Arg4 = {"maxMemory", iocshArgInt};
static const iocshArg config2Arg5 = {"priority", iocshArgInt};
static const iocshArg config2Arg6 = {"stackSize", iocshArgInt};
static const iocshArg * const config2Args[] = {&config2Arg0,
                                              &config2Arg1,
                                              &config2Arg2,
                                              &config2Arg3,
                                              &config2Arg4,
                                              &config2Arg5,
                                              &config2Arg6};
static const iocshFuncDef configADEuresys2 = {"ADEuresysConfig2", 7, config2Args};
static void configCallFunc2(const iocshArgBuf *args)
{
    ADEuresysConfig2(args[0].sval, args[1].ival, args[2].ival, args[3].ival,
                     args[4].ival, args[5].ival, args[6].ival);
}

static void ADEuresysRegister(void)
{
    iocshRegister(&configADEuresys, configCallFunc);
    iocshRegister(&configADEuresys2, configCallFunc2);
}

extern "C" {
epicsExportRegistrar(ADEuresysRegister);
}
