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
    // --- 1. NETWORK SETUP (TWO-WAY UDP) ---
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed" << std::endl;
    }

    // FIX: Allow port reuse to prevent "Bind failed" errors
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
    
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(5005); 
    servaddr.sin_addr.s_addr = INADDR_ANY; 

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Socket bind failed" << std::endl;
    }

    struct sockaddr_in destaddr;
    memset(&destaddr, 0, sizeof(destaddr));
    destaddr.sin_family = AF_INET;
    destaddr.sin_port = htons(5006);
    destaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

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
        // --- UDP COMMAND LISTENER (CHECK 1) ---
        char recvBuffer[32];
        socklen_t len = sizeof(cliaddr);
        if (recvfrom(sockfd, recvBuffer, sizeof(recvBuffer), MSG_DONTWAIT, (struct sockaddr *)&cliaddr, &len) > 0) {
            if (std::string(recvBuffer).find("FFC") != std::string::npos) {
                this->performFFC(); 
            }
        }

        // --- 2. CAMERA READING LOOP ---
        int resets = 0;
        int segmentNumber = -1;
        for(int j=0;j<PACKETS_PER_FRAME;j++) {
            
            // --- UDP COMMAND LISTENER (CHECK 2 - INSIDE PACKET LOOP) ---
            if (j % 20 == 0) {
                if (recvfrom(sockfd, recvBuffer, sizeof(recvBuffer), MSG_DONTWAIT, (struct sockaddr *)&cliaddr, &len) > 0) {
                    if (std::string(recvBuffer).find("FFC") != std::string::npos) {
                        this->performFFC(); 
                    }
                }
            }

            read(spi_cs0_fd, result+sizeof(uint8_t)*PACKET_SIZE*j, sizeof(uint8_t)*PACKET_SIZE);
            int packetNumber = result[j*PACKET_SIZE+1];
            if(packetNumber != j) {
                j = -1; resets += 1; usleep(1000);
                if(resets == 750) {
                    SpiClosePort(0); lepton_reboot(); n_wrong_segment = 0;
                    n_zero_value_drop_frame = 0; usleep(750000); SpiOpenPort(0, spiSpeed);
                }
                continue;
            }
            if ((typeLepton == 3) && (packetNumber == 20)) {
                segmentNumber = (result[j*PACKET_SIZE] >> 4) & 0x0f;
                if ((segmentNumber < 1) || (4 < segmentNumber)) break;
            }
        }

        // --- 3. SEGMENT HANDLING ---
        int iSegmentStart = 1;
        int iSegmentStop;
        if (typeLepton == 3) {
            if ((segmentNumber < 1) || (4 < segmentNumber)) { n_wrong_segment++; continue; }
            n_wrong_segment = 0;
            memcpy(shelf[segmentNumber - 1], result, sizeof(uint8_t) * PACKET_SIZE*PACKETS_PER_FRAME);
            if (segmentNumber != 4) continue;
            iSegmentStop = 4;
        } else {
            memcpy(shelf[0], result, sizeof(uint8_t) * PACKET_SIZE*PACKETS_PER_FRAME);
            iSegmentStop = 1;
        }

        // --- 4. EXTRACT RAW DATA AND SEND TO PYTHON ---
        for(int iSegment = iSegmentStart; iSegment <= iSegmentStop; iSegment++) {
            for(int i=0;i<FRAME_SIZE_UINT16;i++) {
                if(i % PACKET_SIZE_UINT16 < 2) continue;
                uint16_t val = (shelf[iSegment - 1][i*2] << 8) + shelf[iSegment - 1][i*2+1];
                if (val == 0) continue;

                int col, row;
                if (typeLepton == 3) {
                    col = (i % PACKET_SIZE_UINT16) - 2 + (myImageWidth / 2) * ((i % (PACKET_SIZE_UINT16 * 2)) / PACKET_SIZE_UINT16);
                    row = i / PACKET_SIZE_UINT16 / 2 + 30 * (iSegment - 1);
                } else {
                    col = (i % PACKET_SIZE_UINT16) - 2;
                    row = i / PACKET_SIZE_UINT16;
                }
                if(row >= 0 && row < myImageHeight && col >= 0 && col < myImageWidth) {
                     rawValuesBuffer[row * myImageWidth + col] = val;
                }
            }
        }

        sendto(sockfd, (const char*)rawValuesBuffer.data(), 
               rawValuesBuffer.size() * sizeof(uint16_t), 
               0, (const struct sockaddr *) &destaddr, sizeof(destaddr));

        // --- 5. UPDATE DEBUG WINDOW (Original Logic Kept) ---
        if ((autoRangeMin == true) || (autoRangeMax == true)) {
            if (autoRangeMin == true) minValue = 65535;
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

        int d_row, d_column;
        uint16_t d_value;
        QRgb d_color;
        for(int iSegment = iSegmentStart; iSegment <= iSegmentStop; iSegment++) {
            int ofsRow = 30 * (iSegment - 1);
            for(int i=0;i<FRAME_SIZE_UINT16;i++) {
                if(i % PACKET_SIZE_UINT16 < 2) continue;
                uint16_t valueFrameBuffer = (shelf[iSegment - 1][i*2] << 8) + shelf[iSegment - 1][i*2+1];
                if (valueFrameBuffer == 0) continue;
                
                d_value = (valueFrameBuffer - minValue) * scale;
                int ofs_r = 3 * d_value + 0; if (colormapSize <= ofs_r) ofs_r = colormapSize - 1;
                int ofs_g = 3 * d_value + 1; if (colormapSize <= ofs_g) ofs_g = colormapSize - 1;
                int ofs_b = 3 * d_value + 2; if (colormapSize <= ofs_b) ofs_b = colormapSize - 1;
                d_color = qRgb(colormap[ofs_r], colormap[ofs_g], colormap[ofs_b]);
                
                if (typeLepton == 3) {
                    d_column = (i % PACKET_SIZE_UINT16) - 2 + (myImageWidth / 2) * ((i % (PACKET_SIZE_UINT16 * 2)) / PACKET_SIZE_UINT16);
                    d_row = i / PACKET_SIZE_UINT16 / 2 + ofsRow;
                } else {
                    d_column = (i % PACKET_SIZE_UINT16) - 2;
                    d_row = i / PACKET_SIZE_UINT16;
                }
                myImage.setPixel(d_column, d_row, d_color);
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
