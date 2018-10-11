#ifndef GUIDANCE_H
#define GUIDANCE_H
class guidance
{
  private:
    PID xyPID;
    PID xzPID;
    PID stabilityPID;
  public:
    void xyUpdate(&FlightState current, cardinalPos target, 
    void xyUpdate(...
    void stabilityUpdate(...
    void setpointUpdate(...
};


#endif
