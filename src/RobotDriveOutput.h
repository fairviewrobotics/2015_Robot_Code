#ifndef SRC_ROBOTDRIVEOUTPUT_H_
#define SRC_ROBOTDRIVEOUTPUT_H_

class RobotDriveOutput: public PIDOutput {
private:
  RobotDrive* m_baseDrive;

public:
  RobotDriveOutput(RobotDrive* baseDrive) {
    m_baseDrive = baseDrive;
  }

  virtual ~RobotDriveOutput() {}

  void PIDWrite(double output) {
    m_baseDrive->Drive(output, 0.0);
  }
};

#endif /* SRC_ROBOTDRIVEOUTPUT_H_ */
