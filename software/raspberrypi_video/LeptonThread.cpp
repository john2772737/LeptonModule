#include <iostream>
#include <vector>

#include "LeptonThread.h"
#include "Palettes.h"
#include "SPI.h"
#include "Lepton_I2C.h"

// Networking Headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_FRAME)

LeptonThread::LeptonThread() : QThread()
{
    loglevel = 0;
    typeColormap = 3; 
    selectedColormap = colormap_ironblack;
    selectedColormapSize = get_size_colormap_ironblack();
    typeLepton = 2; 
    myImageWidth = 80;
    myImageHeight = 60;
    spiSpeed = 20 * 1000 * 1000; 
    autoRangeMin = true;
    autoRangeMax = true;
    rangeMin = 30000;
    rangeMax = 32000;
}

LeptonThread::~LeptonThread() {
}

void LeptonThread::setLogLevel(uint16_t newLoglevel)
{
    loglevel = newLoglevel;
}

void LeptonThread::useColormap(int newTypeColormap)
{
    switch (newTypeColormap) {
    case 1:
        typeColormap = 1;
        selectedColormap = colormap_rainbow;
        selectedColormapSize = get_size_colormap_rainbow();
        break;
    case 2:
        typeColormap = 2;
        selectedColormap = colormap_grayscale;
        selectedColormapSize = get_size_colormap_grayscale();
        break;
    default:
        typeColormap = 3;
        selectedColormap = colormap_ironblack;
        selectedColormapSize = get_size_colormap_ironblack();
        break;
    }
}

void LeptonThread::useLepton(int newTypeLepton)
{
    switch (newTypeLepton) {
    case 3:
        typeLepton = 3;
        myImageWidth = 160;
        myImageHeight = 120;
        break;
    default:
        typeLepton = 2;
        myImageWidth = 80;
        myImageHeight = 60;
    }
}

void LeptonThread::useSpiSpeedMhz(unsigned int newSpiSpeed)
{
    spiSpeed = newSpiSpeed * 1000 * 1000;
}

void LeptonThread::setAutomaticScalingRange()
{
    autoRangeMin = true;
    autoRangeMax = true;
}

void LeptonThread::useRangeMinValue(uint16_t newMinValue)
{
    autoRangeMin = false;
    rangeMin = newMinValue;
}

void LeptonThread::useRangeMaxValue(uint16_t newMaxValue)
{
    autoRangeMax = false;
    rangeMax = newMaxValue;
}

void LeptonThread::run()
{
    // --- 1. NETWORK SETUP ---
    int sockfd;
    struct sockaddr_in servaddr;
    
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed" << std::endl;
    }
    
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(5005); 
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Localhost
    // ------------------------

    // Define the buffer for sending data (160x120 pixels)
    std::vector<uint16_t> rawValuesBuffer(160 * 120, 0);

    myImage = QImage(myImageWidth, myImageHeight, QImage::Format_RGB888);

    const int *colormap = selectedColormap;
    const int colormapSize = selectedColormapSize;
    uint16_t minValue = rangeMin;
    uint16_t maxValue = rangeMax;
    float diff = maxValue - minValue;
    float scale = 255/diff;
    uint16_t n_wrong_segment = 0;
    uint16_t n_zero_value_drop_frame = 0;

    SpiOpenPort(0, spiSpeed);

    while(true) {

        // --- 2. CAMERA READING LOOP (RESTORED) ---
        int resets = 0;
        int segmentNumber = -1;
        for(int j=0;j<PACKETS_PER_FRAME;j++) {
            read(spi_cs0_fd, result+sizeof(uint8_t)*PACKET_SIZE*j, sizeof(uint8_t)*PACKET_SIZE);
            int packetNumber = result[j*PACKET_SIZE+1];
            if(packetNumber != j) {
                j = -1;
                resets += 1;
                usleep(1000);
                if(resets == 750) {
                    SpiClosePort(0);
                    lepton_reboot();
                    n_wrong_segment = 0;
                    n_zero_value_drop_frame = 0;
                    usleep(750000);
                    SpiOpenPort(0, spiSpeed);
                }
                continue;
            }
            if ((typeLepton == 3) && (packetNumber == 20)) {
                segmentNumber = (result[j*PACKET_SIZE] >> 4) & 0x0f;
                if ((segmentNumber < 1) || (4 < segmentNumber)) {
                    // log_message(10, "[ERROR] Wrong segment number " + std::to_string(segmentNumber));
                    break;
                }
            }
        }
        if(resets >= 30) {
            // log_message(3, "done reading, resets: " + std::to_string(resets));
        }

        // --- 3. SEGMENT HANDLING ---
        int iSegmentStart = 1;
        int iSegmentStop;
        if (typeLepton == 3) {
            if ((segmentNumber < 1) || (4 < segmentNumber)) {
                n_wrong_segment++;
                continue;
            }
            if (n_wrong_segment != 0) {
                n_wrong_segment = 0;
            }
            memcpy(shelf[segmentNumber - 1], result, sizeof(uint8_t) * PACKET_SIZE*PACKETS_PER_FRAME);
            if (segmentNumber != 4) {
                continue;
            }
            iSegmentStop = 4;
        }
        else {
            memcpy(shelf[0], result, sizeof(uint8_t) * PACKET_SIZE*PACKETS_PER_FRAME);
            iSegmentStop = 1;
        }

        // --- 4. EXTRACT RAW DATA AND SEND TO PYTHON ---
        // This loop extracts the real temperature values from the camera packets
        for(int iSegment = iSegmentStart; iSegment <= iSegmentStop; iSegment++) {
            for(int i=0;i<FRAME_SIZE_UINT16;i++) {
                if(i % PACKET_SIZE_UINT16 < 2) {
                    continue; // Skip headers
                }

                // Combine 2 bytes into 1 uint16 value
                uint16_t val = (shelf[iSegment - 1][i*2] << 8) + shelf[iSegment - 1][i*2+1];
                
                if (val == 0) continue;

                // Calculate where this pixel belongs in the 160x120 grid
                int col, row;
                if (typeLepton == 3) {
                    col = (i % PACKET_SIZE_UINT16) - 2 + (myImageWidth / 2) * ((i % (PACKET_SIZE_UINT16 * 2)) / PACKET_SIZE_UINT16);
                    row = i / PACKET_SIZE_UINT16 / 2 + 30 * (iSegment - 1);
                } else {
                    col = (i % PACKET_SIZE_UINT16) - 2;
                    row = i / PACKET_SIZE_UINT16;
                }

                // Verify we are inside the image bounds
                if(row >= 0 && row < myImageHeight && col >= 0 && col < myImageWidth) {
                     // Store in our buffer
                     rawValuesBuffer[row * myImageWidth + col] = val;
                }
            }
        }

        // SEND TO PYTHON
        sendto(sockfd, (const char*)rawValuesBuffer.data(), 
               rawValuesBuffer.size() * sizeof(uint16_t), 
               0, (const struct sockaddr *) &servaddr,  
               sizeof(servaddr));
        // ----------------------------------------------


        // --- 5. UPDATE DEBUG WINDOW (OPTIONAL) ---
        // Standard color mapping for the C++ Window
        if ((autoRangeMin == true) || (autoRangeMax == true)) {
            if (autoRangeMin == true) maxValue = 65535;
            if (autoRangeMax == true) maxValue = 0;
            for(int iSegment = iSegmentStart; iSegment <= iSegmentStop; iSegment++) {
                for(int i=0;i<FRAME_SIZE_UINT16;i++) {
                    if(i % PACKET_SIZE_UINT16 < 2) continue;
                    uint16_t value = (shelf[iSegment - 1][i*2] << 8) + shelf[iSegment - 1][i*2+1];
                    if (value == 0) continue;
                    if ((autoRangeMax == true) && (value > maxValue)) maxValue = value;
                    if ((autoRangeMin == true) && (value < minValue)) minValue = value;
                }
            }
            diff = maxValue - minValue;
            scale = 255/diff;
        }

        int row, column;
        uint16_t value;
        QRgb color;
        for(int iSegment = iSegmentStart; iSegment <= iSegmentStop; iSegment++) {
            int ofsRow = 30 * (iSegment - 1);
            for(int i=0;i<FRAME_SIZE_UINT16;i++) {
                if(i % PACKET_SIZE_UINT16 < 2) continue;
                uint16_t valueFrameBuffer = (shelf[iSegment - 1][i*2] << 8) + shelf[iSegment - 1][i*2+1];
                if (valueFrameBuffer == 0) continue;
                
                value = (valueFrameBuffer - minValue) * scale;
                int ofs_r = 3 * value + 0; if (colormapSize <= ofs_r) ofs_r = colormapSize - 1;
                int ofs_g = 3 * value + 1; if (colormapSize <= ofs_g) ofs_g = colormapSize - 1;
                int ofs_b = 3 * value + 2; if (colormapSize <= ofs_b) ofs_b = colormapSize - 1;
                color = qRgb(colormap[ofs_r], colormap[ofs_g], colormap[ofs_b]);
                
                if (typeLepton == 3) {
                    column = (i % PACKET_SIZE_UINT16) - 2 + (myImageWidth / 2) * ((i % (PACKET_SIZE_UINT16 * 2)) / PACKET_SIZE_UINT16);
                    row = i / PACKET_SIZE_UINT16 / 2 + ofsRow;
                }
                else {
                    column = (i % PACKET_SIZE_UINT16) - 2;
                    row = i / PACKET_SIZE_UINT16;
                }
                myImage.setPixel(column, row, color);
            }
        }
        emit updateImage(myImage);
    }
    
    SpiClosePort(0);
}

void LeptonThread::performFFC() {
    lepton_perform_ffc();
}

void LeptonThread::log_message(uint16_t level, std::string msg)
{
    if (level <= loglevel) {
        std::cerr << msg << std::endl;
    }
}
