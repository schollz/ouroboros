#ifndef OBERHEIMLPF_H
#define OBERHEIMLPF_H

#include <math.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

static float mydsp_faustpower2_f(float value) { return value * value; }

class LPF {
 private:
  int fSampleRate;
  float fConst0;
  float fConst1;
  float fConst2;
  float fHslider0;
  float fRec3[2];
  float fConst3;
  float fHslider1;
  float fRec1[2];
  float fRec2[2];

 public:
  LPF() {}

  int getNumInputs() { return 1; }
  int getNumOutputs() { return 1; }

  static void classInit(int sample_rate) {}

  void instanceConstants(int sample_rate) {
    fSampleRate = sample_rate;
    fConst0 =
        std::min<float>(1.92e+05f, std::max<float>(1.0f, float(fSampleRate)));
    fConst1 = 44.1f / fConst0;
    fConst2 = 1.0f - fConst1;
    fConst3 = 6.2831855f / fConst0;
  }

  void instanceResetUserInterface() {
    fHslider0 = float(0.5f);
    fHslider1 = float(1.0f);
  }

  void instanceClear() {
    for (int l0 = 0; l0 < 2; l0 = l0 + 1) {
      fRec3[l0] = 0.0f;
    }
    for (int l1 = 0; l1 < 2; l1 = l1 + 1) {
      fRec1[l1] = 0.0f;
    }
    for (int l2 = 0; l2 < 2; l2 = l2 + 1) {
      fRec2[l2] = 0.0f;
    }
  }

  void init(int sample_rate) {
    classInit(sample_rate);
    instanceInit(sample_rate);
  }

  void instanceInit(int sample_rate) {
    instanceConstants(sample_rate);
    instanceResetUserInterface();
    instanceClear();
  }

  int getSampleRate() { return fSampleRate; }

  void SetFreq(float fraction) {
    if (fraction > 0 && fraction <= 1) {
      fHslider0 = fraction;
    }
  }
  // void buildUserInterface(UI* ui_interface) {
  // 	ui_interface->openVerticalBox("oberheimLPF");
  // 	ui_interface->addHorizontalSlider("Q", &fHslider1, float(1.0f),
  // float(0.5f), float(1e+01f), float(0.01f));
  // 	ui_interface->addHorizontalSlider("freq", &fHslider0, float(0.5f),
  // float(0.0f), float(1.0f), float(0.001f)); 	ui_interface->closeBox();
  // }

  void Process(int count, float* input0) {
    float fSlow0 = fConst1 * float(fHslider0);
    float fSlow1 = 1.0f / float(fHslider1);
    for (int i0 = 0; i0 < count; i0 = i0 + 1) {
      fRec3[0] = fSlow0 + fConst2 * fRec3[1];
      float fTemp0 =
          std::tan(fConst3 * std::pow(1e+01f, 3.0f * fRec3[0] + 1.0f));
      float fTemp1 = fSlow1 + fTemp0;
      float fTemp2 = fTemp0 * fTemp1 + 1.0f;
      float fTemp3 = float(input0[i0]) - (fRec1[1] + fRec2[1] * fTemp1);
      float fTemp4 = fTemp0 * fTemp3 / fTemp2;
      float fTemp5 =
          std::max<float>(-1.0f, std::min<float>(1.0f, fRec2[1] + fTemp4));
      float fTemp6 = 1.0f - 0.33333334f * mydsp_faustpower2_f(fTemp5);
      float fTemp7 = fTemp0 * fTemp5 * fTemp6;
      float fRec0 = fRec1[1] + fTemp7;
      fRec1[0] = fRec1[1] + 2.0f * fTemp7;
      float fTemp8 = fTemp5 * fTemp6;
      fRec2[0] = fTemp4 + fTemp8;
      input0[i0] = float(fRec0);
      fRec3[1] = fRec3[0];
      fRec1[1] = fRec1[0];
      fRec2[1] = fRec2[0];
    }
  }
};

#endif
