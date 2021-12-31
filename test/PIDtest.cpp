const float PitchPgain = 1;
const float PitchIgain = 0;
const float PitchDgain = 1;

float PitchProportional;
float PitchIntegral;
float PitchDerivative;
float PitchError;
float PrevPitchError;

float PitchOutput;

float RCpitch; //not new
float pitch;   //not new

float prevPitch;
float pitchChange;
float pitchChangeMultiplier = 10;

//need to add prevPitch right before pitch and calculate pitch change
void PitchPID()
{

    PrevPitchError = PitchError;

    PitchError = RCpitch - pitchChange * pitchChangeMultiplier;

    PitchProportional = PitchError * PitchPgain;

    if (PitchIntegral <= 80 && PitchIntegral >= -80)
    {
        PitchIntegral += PitchError * PitchIgain;
    }

    PitchDerivative = (PitchError - PrevPitchError) * PitchDgain;

    PitchOutput = PitchProportional + PitchIntegral + PitchDerivative;
}