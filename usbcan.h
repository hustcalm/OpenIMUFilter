#ifndef USBCAN_H
#define USBCAN_H

#include <Windows.h>
//#include <windef.h>
//#include <minwindef.h>
#include <iostream>
#include "ControlCAN.h"

class USBCAN
{
public:
    USBCAN();
    ~USBCAN();

    USBCAN(DWORD nDevType, DWORD nDevIndex, DWORD nReserved);

    /**
     * @brief openDevice
     * @param nDevType
     * @param nDevIndex
     * @param reserved
     * @return
     */
    bool openDevice();

    /**
     * @brief closeDevice
     * @param nDevType
     * @param DevIndex
     * @return
     */
    bool closeDevice();

    /**
     * @brief initCAN
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @param pInitConfig
     * @return
     */
    bool initCAN(PVCI_INIT_CONFIG pInitConfig);

    /**
     * @brief startCAN
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @return
     */
    bool startCAN();

    /**
     * @brief getReceiveNumber
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @return
     */
    bool getReceiveNumber();

    /**
     * @brief clearBuffer
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @return
     */
    bool clearBuffer();

    /**
     * @brief transmitData
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @param pSend
     * @param len
     * @return
     */
    bool transmitData(PVCI_CAN_OBJ pSend, ULONG len);

    /**
     * @brief transmitData
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @param pReceive
     * @param len
     * @param waitTime
     * @return
     */
    bool receiveData(PVCI_CAN_OBJ pReceive, ULONG len, INT waitTime=-1);

    /**
     * @brief resetCAN
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @return
     */
    bool resetCAN();

    /**
     * @brief readBoardInfo
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @param pInfo
     * @return
     */
    bool readBoardInfo(PVCI_BOARD_INFO pInfo);

    /**
     * @brief readErrorInfo
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @param pErrInfo
     * @return
     */
    bool readErrorInfo(PVCI_ERR_INFO pErrInfo);

    /**
     * @brief readCANStatus
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @return
     */
    bool readCANStatus(PVCI_CAN_STATUS pCANStatus);

    /**
     * @brief getReference
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @param refType
     * @param pData
     * @return
     */
    bool getReference(DWORD refType, PVOID pData);

    /**
     * @brief setReference
     * @param nDevType
     * @param nDevIndex
     * @param nCANIndex
     * @param refType
     * @param pData
     * @return
     */
    bool setReference(DWORD refType, PVOID pData);

    /**
     * @brief showErrorInfo
     */
    void showErrorInfo();

    /**
     * @brief showBoardInfo
     */
    void showBoardInfo();

    /**
     * @brief showCANStatus
     */
    void showCANStatus();

    /**
     * @brief getUnreadFramesNumber
     * @return
     */
    ULONG getUnreadFramesNumber(){
        return this->_unReadFramesNumber;
    }

    /**
     * @brief getTransmittedFramesNumber
     * @return
     */
    ULONG getTransmittedFramesNumber(){
        return this->_transmittedFramesNumber;
    }

    /**
     * @brief getReveivedFramsNumber
     * @return
     */
    ULONG getReveivedFramsNumber(){
        return this->_receivedFramesNumber;
    }

    /**
     * @brief getLastReturnCode
     * @return
     */
    DWORD getLastReturnCode(){
        return this->_retCode;
    }

public:
    // Device Index
    const static DWORD _nDefaultDevType = VCI_USBCAN1;
    const static DWORD _nDefaultDevIndex = 0;
    const static DWORD _nDefaultCANIndex = -1;
    const static DWORD _nDefaultReserved = 0;

private:
    DWORD _nDevType;
    DWORD _nDevIndex;
    DWORD _nCANIndex;
    // Varies of Structures
    PVCI_BOARD_INFO _pBoardInfo;
    PVCI_INIT_CONFIG _pInitConfig;
    PVCI_CAN_STATUS _pCANStatus;
    PVCI_ERR_INFO _pErrorInfo;
    PVCI_FILTER_RECORD _pFilterRecord;

    // Reference Type for setReference & getReference
    DWORD _nRefType;

    //　Buffer for receiving and sending CAN Frames
    PVCI_CAN_OBJ _pVCO;
    VCI_CAN_OBJ _vco[1000];

    // Check if the USBCAN device is opened or closed
    bool _isOpened;
    bool _isClosed;

    // Check if the USBCAN device is initialized
    bool _isInitialized;

    // Check if the USBCAN device is started
    bool _isStarted;

    // Check if the last read Error Info call is successful
    bool _isErrorInfoReady;

    // Number of frames unread in the USBCAN device
    ULONG _unReadFramesNumber;

    // Number of frames reveived in one receive call
    ULONG _receivedFramesNumber;

    // Number of frames transmitted in one receive call
    ULONG _transmittedFramesNumber;

    // Return code of normal API invokes
    DWORD _retCode;

    // Reseverd
    DWORD _nReserved;

};

#endif // USBCAN_H
