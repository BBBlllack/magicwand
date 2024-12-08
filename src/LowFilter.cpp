
#include <LowFilter.hpp>

float filter(float oldValue, float newValue){
    return oldValue * alpha + newValue * (1 - alpha);
}
