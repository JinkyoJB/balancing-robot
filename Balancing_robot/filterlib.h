#ifndef _filterlib_h
#define _filterlib_h
template <typename T>
class LPFilter{
  public:
  LPFilter<T>(float cutoffFreq, float dt);
  T apply(T sample);
  T getAlpha();
 private:
  float alpha, beta;
  T _lastOut;
};
template <typename T>
LPFilter<T>::LPFilter(float cutoffFreq, float dt){
  float Tconst=0.5/cutoffFreq/PI;
  alpha=exp(-dt/Tconst);
  beta=1-alpha;
}
template <typename T>
T LPFilter<T>::apply(T sample){
  _lastOut=alpha*_lastOut+beta*sample;
  return _lastOut;
}
template <typename T>
T LPFilter<T>::getAlpha(){
  return alpha;
}


////
template <typename T>
class HPFilter{
  public:
  HPFilter<T>(float cutoffFreq, float dt);
  T apply(T sample);
 private:
  float alpha;
  T _lastOut;
  T _lastIn;
};
template <typename T>
HPFilter<T>::HPFilter(float cutoffFreq, float dt){
  float Tconst=0.5/cutoffFreq/PI;
  alpha=exp(-dt/Tconst);
  _lastIn=0;
}
template <typename T>
T HPFilter<T>::apply(T sample){
  _lastOut=alpha*_lastOut+sample-_lastIn;
  _lastIn=sample;
  return _lastOut;
}

#endif
