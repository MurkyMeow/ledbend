#include <Arduino.h>

class StepDebounce {
    public:
        uint8_t lastTime;
        bool lastState;

        bool isChanged(uint8_t currentTime, bool currentState) {
            if (lastState != currentState) {
                lastState = currentState;
                lastTime = currentTime;
                return false;
            }

            if (lastState == currentState && currentTime - lastTime > 50) {
                lastTime = currentTime;
                return true;
            }

            return false;
        }
};
