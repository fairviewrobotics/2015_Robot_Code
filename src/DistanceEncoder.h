#ifndef SRC_DISTANCEENCODER_H_
#define SRC_DISTANCEENCODER_H_

class DistanceEncoder {
private:
  Encoder* m_baseEncoder;

public:
  DistanceEncoder(Encoder *baseEncoder) {
	m_baseEncoder = baseEncoder;
  }

  virtual ~DistanceEncoder() {}

  double Get() {
  	return m_baseEncoder->GetDistance();
  }
};

#endif
