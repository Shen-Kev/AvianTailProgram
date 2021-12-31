float pitchValues[10];
float pitch;
float prevPitch;
int derivativePitchIndex = 0;
int derivativePrevPitchIndex = 0;

void loop()
{
    if (derivativePitchIndex < 10)
    {
        derivativePitchIndex++;
    }
    else
    {
        derivativePitchIndex = 0;
    }
    pitchValues[derivativePitchIndex] = pitch;

    if(derivativePitchIndex == 9) {
        derivativePrevPitchIndex = 0;
    }
    else {
        derivativePrevPitchIndex = derivativePitchIndex + 1;
    }
    prevPitch = pitchValues[derivativePrevPitchIndex];
}