#ifndef SRC_DISTANCEENCODER_H_
#define SRC_DISTANCEENCODER_H_

class DistanceEncoder: public PIDSource {
private:
  Encoder* m_baseEncoder;

public:
  DistanceEncoder(Encoder *baseEncoder) {
	m_baseEncoder = baseEncoder;
  }

  virtual ~DistanceEncoder() {}

  double PIDGet() {
	return m_baseEncoder->GetDistance();
  }
};

#endif
