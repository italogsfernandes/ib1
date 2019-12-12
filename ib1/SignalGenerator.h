#ifndef __SignalGenerator__
#define __SignalGenerator__

#include <avr/pgmspace.h> //for IDE versions below 1.0 (2011)
#include "dados_gerados.h"

//Tipos de ondas possiveis de se selecionar
typedef enum {
  REPOUSO_WAVE,
  EEG_BASE_WAVE,
  EEG_ODD_WAVE,
} waveforms_t;

/////////////////////////////////////////
//Classe para gerar as formas de onda //
////////////////////////////////////////
class SignalGenerator_t {
  private:
    waveforms_t _waveform; //Acessible by getter and setter

    uint16_t _actual_index; //Index de leitura do vetor de dados
    float _actual_value; //Valor atual
    float _offset;
    float _amplitude;

    uint16_t _qnt_points;
    uint8_t _freq_divider;

    double get_next() {
      switch (_waveform) {
        case REPOUSO_WAVE:
          _actual_value = 0;
        case EEG_BASE_WAVE:
          _actual_value = pgm_read_float_near(eeg_base_wave + _actual_index * _freq_divider);
          break;
        case EEG_ODD_WAVE:
          _actual_value = pgm_read_float_near(eeg_odd_wave + _actual_index * _freq_divider);
          break;
      }
      _actual_value = _actual_value * _amplitude + _offset;
      ++_actual_index %= (_qnt_points / _freq_divider); //Incremento circular
    }

  public:
    SignalGenerator_t () {
      _waveform = REPOUSO_WAVE;
      _actual_index = 0;
      _offset = 0;
      _amplitude = 100;
      _freq_divider = 1;
    }

    void generate_value(float *value_holder) {
      get_next();
      *value_holder = _actual_value;
    }

    ////////////////////////
    //Getters and Setters //
    ////////////////////////

    waveforms_t getWaveform() {
      return _waveform;
    }

    void setWaveform(waveforms_t waveform) {
      _waveform = waveform;
      switch (_waveform) {
        case REPOUSO_WAVE:
          break;
        case EEG_BASE_WAVE:
          _qnt_points = 10;
        case EEG_ODD_WAVE:
          _qnt_points = 10;
          break;
      }
    }

    void setAmplitude(float amplitude) {
      _amplitude = amplitude;
    }

    void setOffset(float offset) {
      _offset = offset;
    }

    void setQntPointsPerInterval(uint16_t qnt_points) {
      _qnt_points = qnt_points;
    }

    void setFreqDivider(uint16_t freq_divider) {
      _freq_divider = freq_divider;
    }
};

#endif
