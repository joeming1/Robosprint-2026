#ifndef ENCODERAUTO_HPP
#define ENCODERAUTO_HPP

float computePID_L(float expectedRate, float actualRate);
float computePID_R(float expectedRate, float actualRate);
void  resetPID();
void  encodermove_auto(long targetPulses_L, long targetPulses_R, bool forward);

#endif
