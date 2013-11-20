#include "usbcan.h"

USBCAN::USBCAN()
{
    // Default constructor, doing nothing
}

USBCAN::USBCAN(DWORD nDevType, DWORD nDevIndex, DWORD nReserved)
{
    // Constructor
    //std::cout << "Constructing USBCAN Device..." << std::endl;

    this->_nDevType = nDevType;
    this->_nDevIndex = nDevIndex;
    this->_nCANIndex = USBCAN::_nDefaultCANIndex;
    this->_nReserved = nReserved;
    this->_isOpened = FALSE;
    this->_isErrorInfoReady = FALSE;

    // Pointer of varies of structures
    this->_pInitConfig = NULL;
    this->_pBoardInfo = NULL;
    this->_pCANStatus = NULL;
    this->_pErrorInfo = NULL;
    this->_pFilterRecord = NULL;

    // Need memory for Error Report
    this->_pErrorInfo = new VCI_ERR_INFO;

    // Pointer to the data buffer
    this->_pVCO = NULL;

    //std::cout << "Device construct complete!" << std::endl;
}

bool USBCAN::openDevice()
{

    if(this->_isOpened){
        return TRUE;
    }
    else{
        DWORD dwRet;
        dwRet = VCI_OpenDevice(this->_nDevType, this->_nDevIndex, this->_nReserved);
        this->_retCode = dwRet;
        if(dwRet == STATUS_OK){
            this->_isOpened = true;
            this->_nCANIndex += 1;

            /*
            std::cout << "Device Opened Successfully!" << std::endl;
            std::cout << "Device Type: " << this->_nDevType << std::endl;
            std::cout << "Device Index: " << this->_nDevIndex << std::endl;
            std::cout << "CAN Index: " << this->_nCANIndex<< std::endl;
            */

            return true;
        }
        else{
            this->readErrorInfo(this->_pErrorInfo);
            return FALSE;
        }
    }
}

bool USBCAN::initCAN(PVCI_INIT_CONFIG pInitConfig)
{
    if(this->_isOpened){
        DWORD dwRet;
        this->_pInitConfig = pInitConfig;
        std::cout << this->_pInitConfig << std::endl;
        std::cout << this->_pInitConfig->AccCode << std::endl;
        std::cout << this->_pInitConfig->AccMask << std::endl;

        dwRet = VCI_InitCAN(this->_nDevType, this->_nDevIndex, this->_nCANIndex, this->_pInitConfig);
        this->_retCode = dwRet;
        if(dwRet == STATUS_OK){
            //std::cout << "Device initialized successfully!" << std::endl;
            this->_isInitialized = TRUE;
            return TRUE;
        }
        else{
            this->readErrorInfo(this->_pErrorInfo);
            return FALSE;
        }
    }
    else{
        std::cout << "Please open the device before initializing!" << std::endl;
        return FALSE;
    }
}

bool USBCAN::startCAN()
{
    if(this->_isInitialized){
        DWORD dwRet;
        dwRet = VCI_StartCAN(this->_nDevType, this->_nDevIndex, this->_nCANIndex);
        this->_retCode = dwRet;
        if(dwRet == STATUS_OK){
           this->_isStarted = TRUE;
            return TRUE;
        }
        else{
            this->readErrorInfo(this->_pErrorInfo);
            return FALSE;
        }
    }
    else{
        return FALSE;
    }
}

bool USBCAN::getReceiveNumber()
{
    if(this->_isInitialized){
        DWORD dwRet;
        dwRet = VCI_GetReceiveNum(this->_nDevType, this->_nDevIndex, this->_nCANIndex);
        this->_retCode = dwRet;
        this->_unReadFramesNumber = dwRet;
        return TRUE;
    }
    else{
        return FALSE;
    }
}

bool USBCAN::receiveData(PVCI_CAN_OBJ pReceive, ULONG len, INT waitTime)
{
    if(this->_isStarted){
        DWORD dwRet;
        dwRet = VCI_Receive(this->_nDevType, this->_nDevIndex, this->_nCANIndex, pReceive, len, waitTime);
        this->_retCode = dwRet;
        if(dwRet == 0xFFFFFFFF){
            return FALSE;
        }
        else{
            this->_receivedFramesNumber = dwRet;
            return TRUE;
        }
    }
    else{
        return FALSE;
    }
}

bool USBCAN::clearBuffer()
{
    if(this->_isInitialized){
        DWORD dwRet;
        dwRet = VCI_ClearBuffer(this->_nDevType, this->_nDevIndex, this->_nCANIndex);
        this->_retCode = dwRet;
        if(dwRet){
            return TRUE;
        }
        else{
            return FALSE;
        }
    }
    else{
        return FALSE;
    }
}

bool USBCAN::resetCAN()
{
    if(this->_isStarted){
        DWORD dwRet;
        dwRet = VCI_ResetCAN(this->_nDevType, this->_nDevIndex, this->_nCANIndex);
        this->_retCode = dwRet;
        if(dwRet){
            return TRUE;
        }
        else{
            return FALSE;
        }
    }
    else{
        return FALSE;
    }
}

bool USBCAN::closeDevice()
{
    if(this->_isClosed){
        return TRUE;
    }
    else{
        DWORD dwRet;
        dwRet = VCI_CloseDevice(this->_nDevType, this->_nDevIndex);
        this->_retCode = dwRet;
        if(dwRet){
            this->_isClosed = TRUE;
            this->_nDevIndex = USBCAN::_nDefaultCANIndex;
            return TRUE;
        }
        else{
            return FALSE;
        }
    }
}

bool USBCAN::readBoardInfo(PVCI_BOARD_INFO pInfo)
{
    if(this->_isInitialized){
        DWORD dwRet;
        dwRet = VCI_ReadBoardInfo(this->_nDevType, this->_nDevIndex, pInfo);
        this->_retCode = dwRet;
        if(dwRet){
            return TRUE;
        }
        else{
            return FALSE;
        }
    }
    else{
        return FALSE;
    }
}

bool USBCAN::readCANStatus(PVCI_CAN_STATUS pCANStatus)
{
    if(this->_isInitialized){
        DWORD dwRet;
        dwRet = VCI_ReadCANStatus(this->_nDevType, this->_nDevIndex, this->_nCANIndex, pCANStatus);
        this->_retCode = dwRet;
        if(dwRet){
            return TRUE;
        }
        else{
            return FALSE;
        }
    }
    else{
        return FALSE;
    }
}

bool USBCAN::getReference(DWORD refType, PVOID pData)
{
    if(this->_isInitialized){
        DWORD dwRet;
        dwRet = VCI_GetReference(this->_nDevType, this->_nDevIndex, this->_nCANIndex, refType, pData);
        this->_retCode = dwRet;
        if(dwRet){
            return TRUE;
        }
        else{
            return FALSE;
        }
    }
    else{
        return FALSE;
    }
}

bool USBCAN::setReference(DWORD refType, PVOID pData)
{
    if(this->_isInitialized){
        DWORD dwRet;
        dwRet = VCI_SetReference(this->_nDevType, this->_nDevIndex, this->_nCANIndex, refType, pData);
        this->_retCode = dwRet;
        if(dwRet){
            return TRUE;
        }
        else{
            return FALSE;
        }
    }
    else{
        return FALSE;
    }
}

bool USBCAN::readErrorInfo(PVCI_ERR_INFO pErrInfo)
{
    DWORD dwRet;
    dwRet = VCI_ReadErrInfo(this->_nDevType, this->_nDevIndex, this->_nCANIndex, pErrInfo);
    this->_retCode = dwRet;
    if(dwRet){
        this->_isErrorInfoReady = TRUE;
        return TRUE;
    }
    else{
        this->_isErrorInfoReady = FALSE;
        return FALSE;
    }
}

void USBCAN::showErrorInfo()
{
    std::cout << "Last return code: " << this->_retCode;
    std::cout << " Error code: " << this->_pErrorInfo->ErrCode;
    std::cout << std::endl;
}

void USBCAN::showBoardInfo()
{
    // Show basic information about USBCAN-I+
}

void USBCAN::showCANStatus()
{
    // Show current status of USBCAN-I+
}

USBCAN::~USBCAN()
{
    if(this->_pVCO){
        delete this->_pVCO;
        this->_pVCO = NULL;
    }

    if(this->_pBoardInfo){
        delete this->_pBoardInfo;
        this->_pBoardInfo = NULL;
    }

    if(this->_pCANStatus){
        delete this->_pCANStatus;
        this->_pCANStatus = NULL;
    }

    if(this->_pErrorInfo){
        delete this->_pErrorInfo;
        this->_pErrorInfo = NULL;
    }

    if(this->_pFilterRecord){
        delete this->_pFilterRecord;
        this->_pFilterRecord = NULL;
    }

    if(this->_pInitConfig){
        delete this->_pInitConfig;
        this->_pInitConfig = NULL;
    }
}
