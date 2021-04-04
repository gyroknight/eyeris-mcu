#if !defined(__AUDIOCONTROLLER_H__)
#define __AUDIOCONTROLLER_H__

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

typedef struct waveHeader {
  uint32_t chunkID;
  uint32_t chunkSize;
  uint32_t format;
  uint32_t subchunk1ID;
  uint32_t subchunk1Size;
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint32_t byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample;
  uint32_t subchunk2ID;
  uint32_t subchunk2Size;
} waveHeader;

// Forward declarations
typedef struct _snd_pcm snd_pcm_t;
typedef struct _snd_pcm_hw_params snd_pcm_hw_params_t;

class AudioController {
 public:
  AudioController();
  ~AudioController();

  void playFile(const std::string& path);
  int loadFile(const std::string& path);

 private:
  int parseWaveHeader(const waveHeader* header);

  snd_pcm_t* pcmHandle;
  snd_pcm_hw_params_t* hwParams;
  std::unordered_map<std::string, std::shared_ptr<std::vector<uint8_t>>>
      soundFiles;
  uint16_t numChannels;
  uint32_t sampleRate;
};

#endif  // __AUDIOCONTROLLER_H__