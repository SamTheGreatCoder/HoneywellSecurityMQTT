#include "digitalDecoder.h"
#include "analogDecoder.h"
#include "mqtt.h"
//#include "mqtt_config.h"

#include <rtl-sdr.h>

#include <iostream>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <sys/time.h>
#include <cstdlib> // For std::getenv
#include <string>

//class Mqtt {
//public:
    //Mqtt(const std::string& clientId, const std::string& host, int port, const std::string& username, const std::string& password, const std::string& topic, const std::string& status)
        //: clientId(clientId), host(host), port(port), username(username), password(password), topic(topic), status(status) {}

    //void connect() {
        // Implementation of the connect method
    //}

//private:
    //std::string clientId;
    //std::string host;
    //int port;
    //std::string username;
    //std::string password;
    //std::string topic;
    //std::string status;
//};

DigitalDecoder* dDecoder = nullptr; // Declare as a pointer
float magLut[0x10000];

void alarmHandler(int signal)
{
    if (dDecoder) {
        dDecoder->setRxGood(false);
    }
}

int main()
{

    const char* mqttHost = std::getenv("MQTT_HOST");
    const char* mqttPort = std::getenv("MQTT_PORT");
    const char* mqttUsername = std::getenv("MQTT_USERNAME");
    const char* mqttPassword = std::getenv("MQTT_PASSWORD");

    if (!mqttHost || !mqttPort || !mqttUsername || !mqttPassword) {
        std::cerr << "One or more environment variables are not set." << std::endl;
        return 1;
    }

    int port = std::stoi(mqttPort);

    Mqtt mqtt("sensors345", mqttHost, port, mqttUsername, mqttPassword, "/security/sensors345/rx_status", "FAILED");
    dDecoder = new DigitalDecoder(mqtt); // Initialize here
    //DigitalDecoder dDecoder(mqtt);
    AnalogDecoder aDecoder;

    //
    // Open the device
    //
    if(rtlsdr_get_device_count() < 1)
    {
        std::cout << "Could not find any devices" << std::endl;
        return -1;
    }
        
    rtlsdr_dev_t *dev = nullptr;
        
    if(rtlsdr_open(&dev, 0) < 0)
    {
        std::cout << "Failed to open device" << std::endl;
        return -1;
    }
    
    //
    // Set the frequency
    //
    if(rtlsdr_set_center_freq(dev, 345000000) < 0)
    {
        std::cout << "Failed to set frequency" << std::endl;
        return -1;
    }
    
    std::cout << "Successfully set the frequency to " << rtlsdr_get_center_freq(dev) << std::endl;
    
    //
    // Set the gain
    //
    if(rtlsdr_set_tuner_gain_mode(dev, 1) < 0)
    {
        std::cout << "Failed to set gain mode" << std::endl;
        return -1;
    }
    
    const char* gainEnv = std::getenv("RTLSDR_GAIN");
    int gain = gainEnv ? std::stoi(gainEnv) : 350; // Default to 350 if not set

    if(rtlsdr_set_tuner_gain(dev, gain) < 0)
    {
        std::cout << "Failed to set gain" << std::endl;
        return -1;
    }
    
    std::cout << "Successfully set gain to " << rtlsdr_get_tuner_gain(dev) << std::endl;
    
    //
    // Set the sample rate
    //
    if(rtlsdr_set_sample_rate(dev, 1000000) < 0)
    {
        std::cout << "Failed to set sample rate" << std::endl;
        return -1;
    }
    
    std::cout << "Successfully set the sample rate to " << rtlsdr_get_sample_rate(dev) << std::endl;
    
    //
    // Prepare for streaming
    //
    rtlsdr_reset_buffer(dev);
    
    for(uint32_t ii = 0; ii < 0x10000; ++ii)
    {
        uint8_t real_i = ii & 0xFF;
        uint8_t imag_i = ii >> 8;
        
        float real = (((float)real_i) - 127.4) * (1.0f/128.0f);
        float imag = (((float)imag_i) - 127.4) * (1.0f/128.0f);
        
        float mag = std::sqrt(real*real + imag*imag);
        magLut[ii] = mag;
    }
    
    //
    // Common Receive
    //
    
    //aDecoder.setCallback(& { dDecoder->handleData(data); });
    aDecoder.setCallback([&](char data){dDecoder->handleData(data);});
    
    //
    // Async Receive
    //
    
    typedef void(*rtlsdr_read_async_cb_t)(unsigned char *buf, uint32_t len, void *ctx);
    
    auto cb = [](unsigned char *buf, uint32_t len, void *ctx)
    {
        AnalogDecoder *adec = (AnalogDecoder *)ctx;
        
        int n_samples = len/2;
        for(int i = 0; i < n_samples; ++i)
        {
            float mag = magLut[*((uint16_t*)(buf + i*2))];
            adec->handleMagnitude(mag);
        }
    };

    // Setup watchdog to check for a common-mode failure (e.g. antenna disconnection)
    std::signal(SIGALRM, alarmHandler);
  
    // Initialize RX state to good
    //dDecoder.setRxGood(true);
    dDecoder->setRxGood(true);
    const int err = rtlsdr_read_async(dev, cb, &aDecoder, 0, 0);
    std::cout << "Read Async returned " << err << std::endl;
   
/*    
    //
    // Synchronous Receive
    //
    static const size_t BUF_SIZE = 1024*256;
    uint8_t buffer[BUF_SIZE];
    
    while(true)
    {
        int n_read = 0;
        if(rtlsdr_read_sync(dev, buffer, BUF_SIZE, &n_read) < 0)
        {
            std::cout << "Failed to read from device" << std::endl;
            return -1;
        }
        
        int n_samples = n_read/2;
        for(int i = 0; i < n_samples; ++i)
        {
            float mag = magLut[*((uint16_t*)(buffer + i*2))];
            aDecoder.handleMagnitude(mag);
        }
    }
*/    
    //
    // Shut down
    //
    rtlsdr_close(dev);
    delete dDecoder; // Clean up
    return 0;
}




