#ifndef ROSMIP_CONTROL_DSM_INTERFACE_H
#define ROSMIP_CONTROL_DSM_INTERFACE_H

namespace hardware_interface {

  class DsmHandle {
  public:
    struct Data {
    Data():
      ok(0) {}
      std::string name;                       ///< The name of the sensor
      bool* ok;
      float* drive_stick;
      float* turn_stick;
    };

    DsmHandle(const Data& data = Data()):
      name_(data.name),
      ok_(data.ok),
      drive_stick_(data.drive_stick),
      turn_stick_(data.turn_stick)
      {}

    std::string getName()  const {return name_;}
    const bool* getOk()    const { return ok_; }

  private:
    std::string name_;
    bool* ok_;
    float* drive_stick_;
    float* turn_stick_;
  };

  
  class DsmInterface: public HardwareResourceManager<DsmHandle> {};
  
}

#endif // ROSMIP_CONTROL_DSM_INTERFACE_H

