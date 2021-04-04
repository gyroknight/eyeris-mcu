#include "AudioController.hh"

#include <alsa/asoundlib.h>
#include <alsa/error.h>
#include <alsa/pcm.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <ios>
#include <iostream>
#include <iterator>
#include <memory>
#include <ostream>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr const char* pcmDevice = "default";
constexpr snd_pcm_format_t defaultFormat = SND_PCM_FORMAT_S16_LE;
constexpr uint8_t formatSize = sizeof(int16_t);
constexpr int waveChunkHdrSize = 8;
}  // namespace

AudioController::AudioController() : pcmHandle(nullptr), hwParams(nullptr) {
  snd_pcm_hw_params_alloca(&hwParams);
}

AudioController::~AudioController() {
  snd_pcm_close(pcmHandle);
  snd_pcm_hw_params_free(hwParams);
}

void AudioController::playFile(const std::string& path) {
  if (soundFiles.find(path) != soundFiles.end()) {
    std::shared_ptr<std::vector<uint8_t>> file = soundFiles[path];
    waveHeader& header = *reinterpret_cast<waveHeader*>(file->data());
    int ret;
    if ((ret =
             snd_pcm_open(&pcmHandle, pcmDevice, SND_PCM_STREAM_PLAYBACK, 0))) {
      std::cout << "Failed to open PCM handle: " << snd_strerror(ret)
                << std::endl;
      return;
    }

    if ((ret = snd_pcm_hw_params_any(pcmHandle, hwParams))) {
      std::cout << "Failed to fill params for PCM: " << snd_strerror(ret)
                << std::endl;
      return;
    }

    if ((ret = snd_pcm_hw_params_set_access(pcmHandle, hwParams,
                                            SND_PCM_ACCESS_RW_INTERLEAVED))) {
      std::cout << "Failed to interleaved mode: " << snd_strerror(ret)
                << std::endl;
      return;
    }

    if ((ret = snd_pcm_hw_params_set_format(pcmHandle, hwParams,
                                            defaultFormat))) {
      std::cout << "Failed to set sound format to s16l: " << snd_strerror(ret)
                << std::endl;
      return;
    }

    if ((ret = snd_pcm_hw_params_set_channels(pcmHandle, hwParams,
                                              header.numChannels))) {
      std::cout << "Failed to set channel count to " << header.numChannels
                << ": " << snd_strerror(ret) << std::endl;
      return;
    }

    ret = snd_pcm_hw_params_set_rate_near(
        pcmHandle, hwParams,
        reinterpret_cast<unsigned int*>(
            const_cast<uint32_t*>(&header.sampleRate)),
        0);
    if (ret) {
      std::cout << "Failed to set sample rate: " << snd_strerror(ret)
                << std::endl;
      return;
    }

    snd_pcm_nonblock(pcmHandle, 0);  // Set to blocking operation

    if ((ret = snd_pcm_hw_params(pcmHandle, hwParams))) {
      std::cout << "Failed to set hardware params: " << snd_strerror(ret)
                << std::endl;
      return;
    }

    // Get how many frames in a single period
    snd_pcm_uframes_t frames = 0;
    snd_pcm_hw_params_get_period_size(hwParams, &frames, 0);

    size_t wordSize = frames * header.numChannels * formatSize;

    for (uint8_t* head = file->data() + sizeof(waveHeader);
         head < file->data() + file->size(); head += wordSize) {
      if ((ret = snd_pcm_writei(pcmHandle, head, frames)) == -EPIPE) {
        std::cout << "XRUN encountered" << std::endl;
        snd_pcm_prepare(pcmHandle);
      } else if (ret < 0) {
        std::cout << "Failed to write to PCM device: " << snd_strerror(ret)
                  << std::endl;
      }
    }

    snd_pcm_drain(pcmHandle);
    snd_pcm_close(pcmHandle);
  } else {
    std::cout << "File " << path << " not loaded" << std::endl;
  }
}

int AudioController::loadFile(const std::string& path) {
  std::shared_ptr<std::vector<uint8_t>> buffer;
  if (soundFiles.find(path) != soundFiles.end()) {
    // File is already in map
    buffer = soundFiles[path];
  } else {
    // Load file
    std::ifstream file(path.c_str(), std::ios_base::in | std::ios_base::binary);
    if (!file.good()) {
      std::cout << "Failed to open file " << path << std::endl;
      return -ENOENT;
    }

    file.unsetf(std::ios::skipws);  // Disable whitespace skipping

    std::streampos filesize;
    file.seekg(0, std::ios::end);
    filesize = file.tellg();
    file.seekg(0, std::ios::beg);

    if (filesize < static_cast<ssize_t>(sizeof(waveHeader))) {
      std::cout << "File " << path << " is too small" << std::endl;
      return -EPERM;
    }

    buffer = std::make_shared<std::vector<uint8_t>>();
    buffer->reserve(filesize);

    std::copy(std::istream_iterator<uint8_t>(file),
              std::istream_iterator<uint8_t>(), std::back_inserter(*buffer));
    waveHeader* header = reinterpret_cast<waveHeader*>(buffer->data());
    if (header->chunkSize + waveChunkHdrSize != filesize) {
      std::cout << "WAVE header filesize mismatches with actual filesize"
                << std::endl;
      return -EPERM;
    }

    soundFiles[path] = buffer;
  }

  return parseWaveHeader(reinterpret_cast<const waveHeader*>(buffer->data()));
}

int AudioController::parseWaveHeader(const waveHeader* header) {
  // verify that this is a RIFF file header
  const char* riff = "RIFF";
  const char* chunkID = reinterpret_cast<const char*>(&header->chunkID);
  for (short ii = 0; ii < 4; ii++) {
    if (riff[ii] != chunkID[ii]) {
      printf("Not a RIFF file\n");
      return -EPERM;
    }
  }

  // verify that this is WAVE file
  const char* wave = "WAVE";
  const char* format = reinterpret_cast<const char*>(&header->format);
  for (short ii = 0; ii < 4; ii++) {
    if (wave[ii] != format[ii]) {
      printf("Not a WAVE file\n");
      return -EPERM;
    }
  }

  // verify that this is a PCM WAVE file, not another sampling method
  if (header->subchunk1Size != 16 || header->audioFormat != 1) {
    printf("WAVE file is not PCM\n");
    return -ENOTSUP;
  }

  // verify subchunk IDs
  const char* subchunk1ID = reinterpret_cast<const char*>(&header->subchunk1ID);
  const char* subchunk2ID = reinterpret_cast<const char*>(&header->subchunk2ID);
  const char* fmt = "fmt ";
  const char* data = "data";
  for (short ii = 0; ii < 4; ii++) {
    if (subchunk1ID[ii] != fmt[ii] || subchunk2ID[ii] != data[ii]) {
      printf("Corrupted subchunk ID\n");
      return -EPERM;
    }
  }

  // print out information: number of channels, sample rate, total size
  printf("Number of channels: %d\n", header->numChannels);
  printf("Sample Rate: %d\n", header->sampleRate);
  printf("Total Size: %d\n", header->chunkSize + waveChunkHdrSize);

  return 0;
}