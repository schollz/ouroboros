#ifndef TAPE_H
#define TAPE_H
#include <bitset>

#include "circularbuffer.h"
#include "crossfade.h"
#include "tapehead.h"
#define TAPE_PLAY_HEADS 3

class Tape {
 public:
  enum TapeFlags {
    DO_ERASE,
    TAPE_FLAG_COUNT,
  };
  TapeHead head_rec;
  TapeHead head_play[TAPE_PLAY_HEADS];
  size_t head_play_last_pos = 0;
  size_t buffer_max = 1000;
  size_t buffer_start = 2 * CROSSFADE_LIMIT;
  size_t buffer_end = buffer_start + buffer_max;
  std::bitset<TAPE_FLAG_COUNT> flags;

  void Init(size_t start, size_t end, size_t max);
  void RecordingStart();
  void RecordingStop();
  void RecordingErase();
  bool IsPlaying();
  bool IsPlayingOrFading();
  void PlayingFadeOut();
  size_t PlayingCut(size_t pos);
  void PlayingStart();
  void PlayingReset();
  void PlayingStop();
  void Process(float *buf_tape, CircularBuffer &buf_circular, float *in,
               float *out, size_t samples);
};

#endif