#pragma once
#include <Arduino.h>

// ── LoopTimer
// ───────────────────────────────────────────────────────────────── Measures
// real loop duration and enforces a target sample rate. Usage:
//   LoopTimer lt(10);          // target 10 ms (100 Hz)
//   void loop() {
//     float dt = lt.tick();    // call once at top of loop
//     // ... use dt ...
//     lt.sleep();              // call once at bottom — sleeps remaining budget
//   }
// ─────────────────────────────────────────────────────────────────────────────

struct LoopStats {
  float dt_ms;        // actual loop period this iteration (ms)
  float busy_ms;      // time spent doing work (ms), excl. sleep
  float avg_busy_ms;  // rolling average of busy_ms
  float max_busy_ms;  // worst-case busy_ms since last reset
  uint32_t overruns;  // times loop took longer than target
};

class LoopTimer {
 public:
  explicit LoopTimer(uint32_t target_ms)
      : _target_ms(target_ms),
        _prev_us(0),
        _iter_start_us(0),
        _avg_busy_ms(0),
        _max_busy_ms(0),
        _overruns(0),
        _inited(false) {}

  // Call at the very top of loop(). Returns dt in seconds.
  float tick() {
    uint32_t now = micros();
    if (!_inited) {
      _prev_us = now;
      _iter_start_us = now;
      _inited = true;
    }

    _dt_us = now - _prev_us;
    _prev_us = now;
    _iter_start_us = now;  // start of this iteration's work window
    return _dt_us / 1e6f;
  }

  // Call at the very bottom of loop(). Sleeps remaining budget.
  void sleep() {
    uint32_t busy_us = micros() - _iter_start_us;
    float busy_ms = busy_us / 1000.0f;

    // Rolling average (exponential, α = 0.05)
    _avg_busy_ms = 0.95f * _avg_busy_ms + 0.05f * busy_ms;
    if (busy_ms > _max_busy_ms) _max_busy_ms = busy_ms;

    uint32_t target_us = _target_ms * 1000UL;
    if (busy_us < target_us) {
      delayMicroseconds(target_us - busy_us);
    } else {
      _overruns++;
    }
  }

  // Snapshot current stats — zero-cost if you don't call it
  LoopStats stats() const {
    return {_dt_us / 1000.0f, (micros() - _iter_start_us) / 1000.0f,
            _avg_busy_ms, _max_busy_ms, _overruns};
  }

  void resetStats() {
    _avg_busy_ms = 0;
    _max_busy_ms = 0;
    _overruns = 0;
  }

  uint32_t targetMs() const { return _target_ms; }

 private:
  uint32_t _target_ms;
  uint32_t _prev_us;
  uint32_t _iter_start_us;
  uint32_t _dt_us = 0;
  float _avg_busy_ms;
  float _max_busy_ms;
  uint32_t _overruns;
  bool _inited;
};