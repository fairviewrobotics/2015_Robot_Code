#ifndef SRC_DISTANCEENCODER_H_
#define SRC_DISTANCEENCODER_H_

class DistanceEncoder: PIDSource {
private:
  Encoder* m_baseEncoder;

public:
  DistanceEncoder(Encoder *baseEncoder) {
	m_baseEncoder = baseEncoder;
  }

  double pidGet() {
	return m_baseEncoder->GetDistance();
  }
};

#endif
