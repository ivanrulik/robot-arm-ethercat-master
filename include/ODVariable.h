#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>
#include "ethercat.h"
using namespace std;

enum types{
  BOOLEAN,
  INTEGER8,
  INTEGER16,
  INTEGER32,
  INTEGER64,
  UNSIGNED8,
  UNSIGNED16,
  UNSIGNED32,
  UNSIGNED64,
};
class ODVariable
{
private:
  uint16 slave;
  uint16 index;
  bool valueBool{0};
  int8 valueInt8{0};
  int16 valueInt16{0};
  int32 valueInt32{0};
  int64 valueInt64{0};
  uint8 valueUInt8{0};
  uint16 valueUInt16{0};
  uint32 valueUInt32{0};
  uint64 valueUInt64{0};
  const int sizeBool{sizeof(bool)*8};
  const int sizeInt8{sizeof(int8)*8};
  const int sizeInt16{sizeof(int16)*8};
  const int sizeInt32{sizeof(int32)*8};
  const int sizeInt64{sizeof(int64)*8};
  const int sizeUInt8{sizeof(uint8)*8};
  const int sizeUInt16{sizeof(uint16)*8};
  const int sizeUInt32{sizeof(uint32)*8};
  const int sizeUInt64{sizeof(uint64)*8};
  const int TIME0UT{700000};
  int enumValue;
  int getValue(){
  switch(this->enumValue){
    case BOOLEAN:
      return (int) this->valueBool;
    case INTEGER8:
      return (int) this->valueInt8;
    case INTEGER16:
      return (int) this->valueInt16;
    case INTEGER32:
      return (int) this->valueInt32;
    case INTEGER64:
      return (int) this->valueInt64;
    case UNSIGNED8:
      return (int) this->valueUInt8;
    case UNSIGNED16:
      return (int) this->valueUInt16;
    case UNSIGNED32:
      return (int) this->valueUInt32;
    case UNSIGNED64:
      return (int) this->valueUInt64;  
  }
  }
  int* getSizePointer(){
  switch(this->enumValue){
    case BOOLEAN:
      return &this->sizeBool;
    case INTEGER8:
      return &this->sizeInt8;
    case INTEGER16:
      return &this->sizeInt16;
    case INTEGER32:
      return &this->sizeInt32;
    case INTEGER64:
      return &this->sizeInt64;
    case UNSIGNED8:
      return &this->sizeUInt8;
    case UNSIGNED16:
      return &this->sizeUInt16;
    case UNSIGNED32:
      return &this->sizeUInt32;
    case UNSIGNED64:
      return &this->sizeUInt64;  
  }
  }
  void* getPointerValue(){
  switch(this->enumValue){
    case BOOLEAN:
      return &this->valueBool;
    case INTEGER8:
      return &this->valueInt8;
    case INTEGER16:
      return &this->valueInt16;
    case INTEGER32:
      return &this->valueInt32;
    case INTEGER64:
      return &this->valueInt64;
    case UNSIGNED8:
      return &this->valueUInt8;
    case UNSIGNED16:
      return &this->valueUInt16;
    case UNSIGNED32:
      return &this->valueUInt32;
    case UNSIGNED64:
      return &this->valueUInt64;  
  }
  }
  void setEnumValue(int enumValue){
    this->enumValue = enumValue;
  }
  void setValue(int value){
    switch(this->enumValue){
    case BOOLEAN:
      this->valueBool = (bool) value;
    case INTEGER8:
      this->valueInt8 = (int8) value;
    case INTEGER16:
      this->valueInt16 = (int16) value;
    case INTEGER32:
      this->valueInt32 = (int32) value;
    case INTEGER64:
      this->valueInt64 = (int64) value;
    case UNSIGNED8:
      this->valueUInt8 = (uint8) value;
    case UNSIGNED16:
      this->valueUInt16 = (uint16) value;
    case UNSIGNED32:
      this->valueUInt32 = (uint32) value;
    case UNSIGNED64:
      this->valueUInt64 = (uint64) value;  
  }
  }
public:
  ODVariable(uint16 slave, uint16 index, int enumValue = -1){
  this->slave = slave;
  this->index = index;
  if (enumValue != -1){
      this->enumValue = enumValue;
  }
  }

  int SDOwrite(int value, uint8 subIndex = 0x00, int enumValue = -1){
    int retval = 0;
    if (enumValue != -1){
      this->setEnumValue(enumValue);
    }
    int cont = 0;
    while (retval == 0){
      this->setValue(value);
      retval = ec_SDOwrite(this->slave, this->index, subIndex, FALSE, 
                           *this->getSizePointer(), this->getPointerValue(), this->TIME0UT);
      cont++;
      this->SDOread(subIndex);
      if (cont == 100){
          cout<<"me atore: "<< hex << this->index<<" Value:" << this->getValue() << " Slave: "<< this->slave<<endl;
      }
    }
    cout<<"Index: "<< hex << this->index<<" Value:" << this->getValue()
    << " Slave: "<< this->slave<< " Sub-Index: "<< hex<< subIndex <<endl;
    return retval;
  }

  int SDOread( uint8 subIndex = 0x00,  int enumValue = -1){
      int retval = 0;
      if (enumValue !=-1){
        this->setEnumValue(enumValue);
      }
      int cont = 0;
      while (cont < 10){
          retval = ec_SDOread(this->slave, this->index, subIndex, FALSE, 
                              this->getSizePointer(), this->getPointerValue(), this->TIME0UT);
          cont++;
      }
      return this->getValue();
  }
};