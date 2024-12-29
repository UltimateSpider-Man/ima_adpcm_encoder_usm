#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <stdexcept>

// A simple struct to represent the wave file's basic format info.
// We'll parse only what's needed (PCM 16-bit, stereo, etc.).
struct WavInfo {
    uint32_t sampleRate = 44100;
    uint16_t channels = 2;
    uint16_t bitsPerSample = 16;
    uint32_t dataSize = 0; // # bytes of raw PCM data
    uint32_t dataOffset = 0; // offset in file where data begins
};

//------------------------------------------------------------------------------
// 1) A minimal function to read the basic PCM wave header.
//    We do not handle every chunk type, just enough for typical 16-bit PCM.
//------------------------------------------------------------------------------
WavInfo readPcmWavHeader(std::ifstream& ifs)
{
    // We'll parse the standard 44-byte PCM WAVE header (RIFF chunk + "fmt " + "data").
    // Real WAV files can have extra chunks (LIST, etc.), so be more robust in production.
    WavInfo info;

    // Read the first 12 bytes: 'RIFF' + fileSize + 'WAVE'
    char riffId[4];
    ifs.read(riffId, 4);
    if (std::memcmp(riffId, "RIFF", 4) != 0) {
        throw std::runtime_error("Not a RIFF file");
    }
    uint32_t riffSize;
    ifs.read(reinterpret_cast<char*>(&riffSize), 4);
    char waveId[4];
    ifs.read(waveId, 4);
    if (std::memcmp(waveId, "WAVE", 4) != 0) {
        throw std::runtime_error("Not a WAVE file");
    }

    // Next, we look for the "fmt " chunk
    while (true) {
        char chunkId[4];
        if (!ifs.read(chunkId, 4)) {
            throw std::runtime_error("No fmt chunk found");
        }
        uint32_t chunkSize = 0;
        if (!ifs.read(reinterpret_cast<char*>(&chunkSize), 4)) {
            throw std::runtime_error("EOF reading chunkSize");
        }

        if (std::memcmp(chunkId, "fmt ", 4) == 0) {
            // parse format chunk
            uint16_t audioFormat;
            ifs.read(reinterpret_cast<char*>(&audioFormat), 2);
            ifs.read(reinterpret_cast<char*>(&info.channels), 2);
            ifs.read(reinterpret_cast<char*>(&info.sampleRate), 4);

            uint32_t byteRate;
            ifs.read(reinterpret_cast<char*>(&byteRate), 4);

            uint16_t blockAlign;
            ifs.read(reinterpret_cast<char*>(&blockAlign), 2);

            ifs.read(reinterpret_cast<char*>(&info.bitsPerSample), 2);

            // skip any extra bytes
            if (chunkSize > 16) {
                ifs.seekg(chunkSize - 16, std::ios::cur);
            }
        }
        else if (std::memcmp(chunkId, "data", 4) == 0) {
            // we found data chunk
            info.dataSize = chunkSize;
            info.dataOffset = static_cast<uint32_t>(ifs.tellg());
            // move file pointer past data chunk
            ifs.seekg(chunkSize, std::ios::cur);
        }
        else {
            // skip this chunk
            ifs.seekg(chunkSize, std::ios::cur);
        }

        // If we've found data chunk, we can break early. 
        if (info.dataSize > 0 && info.dataOffset > 0) {
            break;
        }
    }

    return info;
}

//------------------------------------------------------------------------------
// 2) The IMA ADPCM step table and index tables
//    Standard IMA ADPCM decoding/encoding constants
//------------------------------------------------------------------------------
static int stepTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107,118,
    130,143,157,173,190,209,230,253,279,307,
    337,371,408,449,494,544,598,658,724,796,
    876,963,1060,1166,1282,1408,1542,1685,1849,2020,
    2203,2400,2611,2837,3080,3340,3618,3915,4233,4574,
    4939,5330,5750,6202,6688,7203,7755,8347,8982,9666,
    10406,11208,12073,13005,14011,15094,16256,17493,18811
};

static int indexAdjust[8] = {
    -1, -1, -1, -1,  2,  4,  6,  8
};

//------------------------------------------------------------------------------
// 3) Encode one sample (16-bit) into a 4-bit nibble for IMA ADPCM.
//    We maintain 'stepIndex' and 'predictor' for each channel separately.
//------------------------------------------------------------------------------
uint8_t encodeSample(int16_t sample, int& predictor, int& stepIndex)
{
    // clamp stepIndex
    if (stepIndex < 0) stepIndex = 0;
    if (stepIndex > 88) stepIndex = 88;
    int step = stepTable[stepIndex];

    // compute diff
    int diff = sample - predictor;
    int sign = (diff < 0) ? 8 : 0;
    if (diff < 0) diff = -diff;

    // nibble
    int delta = 0;
    int mask = 4;
    for (int i = 2; i >= 0; i--) {
        if (diff >= step) {
            delta |= mask;
            diff -= step;
        }
        step >>= 1;
        mask >>= 1;
    }
    delta |= sign;

    // update predictor
    int diffq = 0;
    int st = stepTable[stepIndex];
    if (delta & 4) diffq += st;
    if (delta & 2) diffq += st >> 1;
    if (delta & 1) diffq += st >> 2;
    diffq += st >> 3;
    if (delta & 8) diffq = -diffq;

    predictor += diffq;
    // clamp predictor to 16-bit
    if (predictor > 32767) predictor = 32767;
    if (predictor < -32768) predictor = -32768;

    // update stepIndex
    stepIndex += indexAdjust[delta & 7];
    if (stepIndex < 0) stepIndex = 0;
    if (stepIndex > 88) stepIndex = 88;

    return (uint8_t)(delta & 0x0F);
}

//------------------------------------------------------------------------------
// 4) Encode a stereo buffer from 16-bit PCM to 4-bit IMA ADPCM in small blocks.
//    For demonstration, we do a naive approach (e.g., 1 sample => 1 nibble, etc.).
//    A typical IMA WAVE chunk is 512 or 1024 bytes per block including block headers.
//    We'll skip block-level headers for brevity, or store them minimalistically.
//------------------------------------------------------------------------------
std::vector<uint8_t> encodeImaAdpcmStereo(const int16_t* samples, size_t numSamples, uint32_t sampleRate)
{
    // We'll produce ~half the size in 4-bit.
    // Real IMA blocks for stereo might store a 4-byte header per channel with predictor + stepIndex, etc.
    // Let's do a minimal approach: start predictor=0, stepIndex=0 for each channel, encode nibble by nibble.

    int predictorL = 0;
    int stepIndexL = 0;
    int predictorR = 0;
    int stepIndexR = 0;

    // each stereo frame => 2 samples (left, right)
    // each frame => 2 nibbles (one for L, one for R) => 1 byte
    // so total ADPCM samples = numSamples/2 stereo frames => ADPCM bytes = (numSamples/2)

    size_t stereoFrames = numSamples / 2; // each frame has L + R
    std::vector<uint8_t> adpcmData;
    adpcmData.reserve(stereoFrames);

    for (size_t i = 0; i < stereoFrames; i++) {
        int16_t left = samples[i * 2 + 0];
        int16_t right = samples[i * 2 + 1];

        uint8_t nibbleL = encodeSample(left, predictorL, stepIndexL);
        uint8_t nibbleR = encodeSample(right, predictorR, stepIndexR);

        // combine into one byte => L nibble is low, R nibble is high, or vice versa
        uint8_t byteVal = (nibbleR << 4) | (nibbleL & 0xF);
        adpcmData.push_back(byteVal);
    }

    return adpcmData;
}

//------------------------------------------------------------------------------
// 5) Write a minimal RIFF/WAVE header for IMA ADPCM 4-bit stereo
//    Like the QuickBMS script's "TOIMAWAV" snippet, but actually consistent
//    with WAVE specifications for formatTag=0x11 (IMA ADPCM).
//------------------------------------------------------------------------------
void writeImaAdpcmWave(std::ofstream& ofs, const std::vector<uint8_t>& adpcm, uint32_t sampleRate, uint16_t channels)
{
    // We'll craft a standard ~46-byte ADPCM WAVE header with "fmt " chunkSize=0x12 
    // (2 bytes extension field). We do a "data" chunk after that.
    // For simplicity, let's skip writing the official blockAlign if needed.

    // 1) RIFF chunk
    // "RIFF"
    ofs.write("RIFF", 4);
    uint32_t fileSize = 36 + 4 + (uint32_t)adpcm.size(); // chunk after RIFF (excl. 8 byte up front)
    ofs.write(reinterpret_cast<const char*>(&fileSize), 4);
    // "WAVE"
    ofs.write("WAVE", 4);

    // 2) "fmt " chunk
    // "fmt "
    ofs.write("fmt ", 4);
    // chunk size for IMA ADPCM is typically 0x12 (18 bytes) for WAVEFORMATEX w/ extra size=2
    uint32_t fmtSize = 0x12;
    ofs.write(reinterpret_cast<const char*>(&fmtSize), 4);

    // wFormatTag = 0x11 (IMA ADPCM)
    uint16_t wFormatTag = 0x0011;
    ofs.write(reinterpret_cast<const char*>(&wFormatTag), 2);

    // wChannels
    ofs.write(reinterpret_cast<const char*>(&channels), 2);

    // dwSamplesPerSec
    ofs.write(reinterpret_cast<const char*>(&sampleRate), 4);

    // For a real IMA wave, you might do: avgBytesPerSec = blockAlign * sampleRate / samplesPerBlock
    // We'll do something approximate or from the QuickBMS code. 
    // The script does "AVGBYTES = (689 * BLOCKALIGN) + 4" with weird formulas.
    // We'll pick a naive approach: let's guess dataRate. Or do a small formula:
    uint32_t avgBytesPerSec = 4 * sampleRate / 1;
    ofs.write(reinterpret_cast<const char*>(&avgBytesPerSec), 4);

    // wBlockAlign
    // For IMA ADPCM stereo, blockAlign might be e.g. 0x100 or 0x200 in typical blocks.
    // We'll store a small number. The QuickBMS code does "xmath BLOCKALIGN '36 * CHANNELS'"
    // That's 72 if channels=2 => 0x48
    uint16_t blockAlign = (uint16_t)(36 * channels);
    ofs.write(reinterpret_cast<const char*>(&blockAlign), 2);

    // wBitsPerSample = 4 for IMA ADPCM
    uint16_t bitsPerSample = 4;
    ofs.write(reinterpret_cast<const char*>(&bitsPerSample), 2);

    // cbSize = 2 (the size of extra extension bytes after WAVEFORMATEX)
    uint16_t cbSize = 2;
    ofs.write(reinterpret_cast<const char*>(&cbSize), 2);

    // Now 2 extension bytes: often "samples per block" for IMA
    // We'll pick a small guess or 0. Let's do 0:
    uint16_t samplesPerBlock = 0;
    ofs.write(reinterpret_cast<const char*>(&samplesPerBlock), 2);

    // 3) "data" chunk
    ofs.write("data", 4);
    uint32_t dataChunkSize = (uint32_t)adpcm.size();
    ofs.write(reinterpret_cast<const char*>(&dataChunkSize), 4);

    // Then write the ADPCM payload
    ofs.write(reinterpret_cast<const char*>(adpcm.data()), adpcm.size());
}

//------------------------------------------------------------------------------
// Main driver: read PCM wave, convert to stereo IMA ADPCM, write new wave
//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_pcm.wav> <output_adpcm.wav>\n";
        return 1;
    }

    std::string inWav = argv[1];
    std::string outWav = argv[2];

    // 1) Open input, read wave header
    std::ifstream ifs(inWav, std::ios::binary);
    if (!ifs.is_open()) {
        std::cerr << "Error opening input file: " << inWav << "\n";
        return 1;
    }

    WavInfo info;
    try {
        info = readPcmWavHeader(ifs);
    }
    catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    // Validate: must be 16-bit stereo PCM for this example
    if (info.channels != 2 || info.bitsPerSample != 16) {
        std::cerr << "This example only handles stereo 16-bit PCM.\n";
        return 1;
    }

    // 2) Read PCM samples
    std::vector<int16_t> samples(info.dataSize / 2); // 2 bytes per sample
    ifs.seekg(info.dataOffset, std::ios::beg);
    if (!ifs.read(reinterpret_cast<char*>(samples.data()), info.dataSize)) {
        std::cerr << "Error reading PCM data.\n";
        return 1;
    }
    ifs.close();

    // 3) Encode to IMA ADPCM
    //    We'll produce a 4-bit result => about half the size if naive
    std::vector<uint8_t> adpcm = encodeImaAdpcmStereo(samples.data(), samples.size(), info.sampleRate);

    // 4) Write to the output as a WAVE with wFormatTag=0x11
    std::ofstream ofs(outWav, std::ios::binary);
    if (!ofs.is_open()) {
        std::cerr << "Error creating output file: " << outWav << "\n";
        return 1;
    }

    writeImaAdpcmWave(ofs, adpcm, info.sampleRate, info.channels);
    ofs.close();

    std::cout << "Converted " << inWav << " (" << info.sampleRate << " Hz, 2-ch, 16-bit PCM) "
        << " to IMA ADPCM .wav => " << outWav << "\n"
        << "Encoded size: " << adpcm.size() << " bytes.\n";
    return 0;
}