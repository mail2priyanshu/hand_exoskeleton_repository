#ifndef MAXON_MOTOR_H
#define MAXON_MOTOR_H
#endif // MAXON_MOTOR_H

#ifndef definitions_H
#include <definitions.h>
#define definitions_H
#endif // definitions_H

#ifndef _H_LINUX_EPOSCMD_
#include "definitions_motor.h"
#define _H_LINUX_EPOSCMD_
#endif // _H_LINUX_EPOSCMD_

void closeDevice(void *keyHandle_)
{
    unsigned int ErrorCode = 0;

    cout<<"Closing Device!"<<endl;

    if(keyHandle_ != 0)
        VCS_CloseDevice(keyHandle_, &ErrorCode);

}

void EnableDevice(void *keyHandle_, unsigned short nodeId)
{
    unsigned int ErrorCode = 0;
    int IsInFault = FALSE;


    // Configuring Analog Input Model
    if( VCS_AnalogInputConfiguration(keyHandle_, nodeId, 1, AIC_ANALOG_POSITION_SETPOINT, 1, &ErrorCode))
    {
        cout<<"Configured position analog input mode!"<<endl;
    }
    else
    {
        cout<<"Failed to configure position analog input mode!, error code="<<ErrorCode<<endl;
    }

    if( VCS_GetFaultState(keyHandle_, nodeId, &IsInFault, &ErrorCode) )
    {
        if( IsInFault && !VCS_ClearFault(keyHandle_, nodeId, &ErrorCode) )
        {
            cout<<"Clear fault failed!, error code="<<ErrorCode<<endl;
            return;
        }

        int IsEnabled = FALSE;
        if( VCS_GetEnableState(keyHandle_, nodeId, &IsEnabled, &ErrorCode) )
        {
            if( !IsEnabled && !VCS_SetEnableState(keyHandle_, nodeId, &ErrorCode) )
            {
                cout<<"Set enable state failed!, error code="<<ErrorCode<<endl;
            }
            else
            {
                cout<<"Set enable state succeeded!"<<endl;
            }
        }
    }
    else
    {
        cout<<"Get fault state failed!, error code="<<ErrorCode<<endl;
    }

    // Enabling Position Analog Input Mode
    if( VCS_EnableAnalogPositionSetpoint(keyHandle_, nodeId, &ErrorCode))
    {
        cout<<"Enabled position analog input mode!"<<endl;
    }
    else
    {
        cout<<"Failed to enable position analog input mode!, error code="<<ErrorCode<<endl;
    }

}

void DisableDevice(void *keyHandle_, unsigned short nodeId)
{
    unsigned int ErrorCode = 0;
    int IsInFault = FALSE;

    // Disabling Position Analog Input Mode
    if( VCS_DisableAnalogPositionSetpoint(keyHandle_, nodeId, &ErrorCode))
    {
        cout<<"Disabled position analog input mode!"<<endl;
    }
    else
    {
        cout<<"Failed to disable position analog input mode!, error code="<<ErrorCode<<endl;
    }


    if( VCS_GetFaultState(keyHandle_, nodeId, &IsInFault, &ErrorCode) )
    {
        if( IsInFault && !VCS_ClearFault(keyHandle_, nodeId, &ErrorCode) )
        {
            cout<<"Clear fault failed!, error code="<<ErrorCode<<endl;
            return;
        }

        int IsEnabled = FALSE;
        if( VCS_GetEnableState(keyHandle_, nodeId, &IsEnabled, &ErrorCode) )
        {
            if( IsEnabled && !VCS_SetDisableState(keyHandle_, nodeId, &ErrorCode) )
            {
                cout<<"Set disable state failed!, error code="<<ErrorCode<<endl;
            }
            else
            {
                cout<<"Set disable state succeeded!"<<endl;
            }
        }
    }
    else
    {
        cout<<"Get fault state failed!, error code="<<ErrorCode<<endl;
    }
}

void Move(void *keyHandle_, long TargetPosition, unsigned short nodeId)
{
    unsigned int ErrorCode = 0;

    //    unsigned int ProfileVelocity = 10000;
    //    unsigned int ProfileAcceleration = 8000;
    //    unsigned int ProfileDeceleration = 8000;

    if( VCS_ActivatePositionMode(keyHandle_, nodeId, &ErrorCode) )
    {
        int Absolute = TRUE;
        int Immediately = TRUE;

        if( !Absolute )
        {
            int PositionIs = 0;

            if( VCS_GetPositionIs(keyHandle_, nodeId, &PositionIs, &ErrorCode) );
        }

        if( !VCS_MoveToPosition(keyHandle_, nodeId, TargetPosition, Absolute, Immediately, &ErrorCode) )
        {
            cout<<"Move to position failed!, Error code="<<ErrorCode<<endl;
        }

    }
    else
    {
        cout<<"Activate profile position mode failed!, Error code="<<ErrorCode<<endl;
    }
}

void Halt(void *keyHandle_, unsigned short nodeId)
{
    unsigned int ErrorCode = 0;

    if( !VCS_HaltPositionMovement(keyHandle_, nodeId, &ErrorCode) )
    {
        cout<<"Halt position movement failed!, error code="<<ErrorCode<<endl;
    }
}

void* activate_device(char *PortName, unsigned short nodeId)
{
    // Configuring EPOS for analog motor control
    char DeviceName[]="EPOS2";
    char ProtocolStackName[] = "MAXON SERIAL V2";
    char InterfaceName[] = "USB";
    unsigned int ErrorCode = 0x00;
    unsigned long timeout_ = 500;
    unsigned long baudrate_ = 1000000;
    void *keyHandle_;

    keyHandle_ = VCS_OpenDevice(DeviceName,ProtocolStackName,InterfaceName,PortName,&ErrorCode);

    if( keyHandle_ == 0 )
    {
        cout<<"Open device failure, error code="<<ErrorCode<<endl;
        exit(0);
    }
    else
    {
        cout<<"Open device success!"<<endl;
    }


    if( !VCS_SetProtocolStackSettings(keyHandle_, baudrate_, timeout_, &ErrorCode) )
    {
        cout<<"Set protocol stack settings failed!, error code="<<ErrorCode<<endl;
        closeDevice(keyHandle_);
        exit(0);
    }

    EnableDevice(keyHandle_, nodeId);
    return keyHandle_;
}
