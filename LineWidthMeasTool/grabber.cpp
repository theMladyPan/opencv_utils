#include "grabber.h"



Grabber::Grabber(uint8_t index)
{
  pSystem = System::GetInstance();
  CameraList camList = pSystem->GetCameras();
  if (camList.GetSize() == 0){
      throw runtime_error("No camera available");
    }

  pCam = camList.GetByIndex(index);
  camList.Clear();
}

Grabber::Grabber()
{

}

Grabber::Grabber(CameraPtr pCamera)
{
    pCam = pCamera;
}

Grabber::Grabber(string serialNr)
{
    pSystem = System::GetInstance();
    CameraList camList = pSystem->GetCameras();
    pCam = camList.GetBySerial(serialNr);
}

Grabber::~Grabber()
{
    this->free();
}

void Grabber::init()
{
    this->init(AcquisitionMode_Continuous);
}

void Grabber::init(AcquisitionModeEnums modeEnum)
{
    pCam->Init();
    INodeMap &nodeMap = pCam->GetNodeMap();
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    int64_t acquisitionMode ;
    switch (modeEnum) {
        case AcquisitionMode_Continuous:{
           acquisitionMode = ptrAcquisitionMode->GetEntryByName("Continuous")->GetValue();
           break;
        }
        case AcquisitionMode_SingleFrame:{
          acquisitionMode = ptrAcquisitionMode->GetEntryByName("SingleFrame")->GetValue();
          break;
        }
        case AcquisitionMode_MultiFrame:{
          acquisitionMode = ptrAcquisitionMode->GetEntryByName("MultiFrame")->GetValue();
          break;
        }
        default:{
          acquisitionMode = ptrAcquisitionMode->GetEntryByName("Continuous")->GetValue();
          }

        }

    ptrAcquisitionMode->SetIntValue(acquisitionMode);
}

void Grabber::start(){
  // TODO: rewrite for single shot
    try {
        if(!pCam->IsInitialized()){
            this->init();
        }

        if(!pCam->IsStreaming()){
            pCam->BeginAcquisition();
        }
    } catch (Spinnaker::Exception &e) {
        std::cerr << e.what();
        throw std::runtime_error("Camera probbably disconnected, please reconnect camera");
    }
}

void Grabber::stop()
{
    if(pCam!= nullptr){
        if(pCam->IsStreaming()){
            pCam->EndAcquisition();
        }
    }
}

string Grabber::getSerialNr(){
  gcstring deviceSerialNumber("");
  INodeMap &nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
  CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
  if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial)) {
    deviceSerialNumber = ptrStringSerial->GetValue();

    return deviceSerialNumber.c_str();
  }
  else{
      return "";
    }
}

ImagePtr Grabber::getResult()
{
  // size_t width = pResultImage->GetWidth();
  // size_t height = pResultImage->GetHeight();
    try {
        lastResult = pCam->GetNextImage();//->Convert(PixelFormat_Mono8, HQ_LINEAR);
    } catch (Spinnaker::Exception &e) {
        std::cerr << e.what();
        throw std::runtime_error("Unable to get frame");
    }
  return lastResult;
}

ImagePtr Grabber::getLastResult()
{
  if(lastResult != nullptr){
      return lastResult;
    }
  else{
      return getResult();
    }
}

void Grabber::saveLast(string filename)
{
  if(lastResult != nullptr){
      lastResult->Save(filename.c_str());
    }else{
      this->getResult()->Save(filename.c_str());
    }
}

void Grabber::free()
{
    try{
        if(pCam != nullptr){
            try{
                this->stop();
            }catch (Spinnaker:: Exception &e){
                std::cerr<<e.GetErrorMessage();
            }
            pCam->DeInit();
            pCam = nullptr;
        }
    }catch(...){
       // handle exception
    }
    /* if(pSystem != nullptr){
      if(pSystem->IsInUse()){
        pSystem->ReleaseInstance();
        }
    }*/
}

uint8_t nOfCamAvail()
{
  SystemPtr system = System::GetInstance();
  system->UpdateCameras();
  CameraList camList = system->GetCameras();
  uint8_t listSize = uint8_t(camList.GetSize());
  camList.Clear();
  if(!system->IsInUse()){
      system->ReleaseInstance();
    }
  return uint8_t(listSize);
}

vector<string> camAvail()
{
  vector<string> cameras;
  int availCameras = nOfCamAvail();
  if(availCameras>0){
      for(uint8_t i=0; i<availCameras;i++){
          Grabber grb = Grabber(i);
          cameras.push_back(grb.getSerialNr());
        }
  }

  return cameras;
}
