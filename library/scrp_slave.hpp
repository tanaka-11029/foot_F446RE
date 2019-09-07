#ifndef SCRP_SLAVE_H
#define SCRP_SLAVE_H
#include "mbed.h"

/*USBでPCにつなぐポートと、基板上でRasPiとつなぐポートを同時に開く。
 *RedePinの有り無しの選択、ポートを一つだけ開くことも可。
 *以下から選択。
 *ScrpSlave(PinName TX1,PinName RX1,uint32_t addr);//RedePinなし、１ポート
 *ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,uint32_t addr);//RedePinあり、１ポート
 *ScrpSlave(PinName TX1,PinName RX1,PinName TX2,PinName RX2,uint32_t addr);//RedePinなし、２ポート
 *ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,PinName TX2,PinName RX2,uint32_t addr);//RedePinあり、１ポート＋RedePinなし、１ポート
 *example not usb port
 *L432KC : TX = PA_9 , RX = PA_10 , REDE = PA_12 , addr = 0x0803e000
 *F446RE : TX = PC_12 , RX = PD_2 , RDDE = PH_1 , addr = 0x0807ffff
 */
//ScrpSlave slave(SERIAL_TX,SERIAL_RX);

inline int constrain(int x,int a,int b){
    return (x < a ? a : x > b ? b : x);
}

inline double constrain(double x,double a,double b){
    return (x < a ? a : x > b ? b : x);
}

class ScrpSlave{
public:
    ScrpSlave(PinName TX1,PinName RX1,uint32_t addr);//RedePinなし、１ポート
    ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,uint32_t addr);//RedePinあり、１ポート
    ScrpSlave(PinName TX1,PinName RX1,PinName TX2,PinName RX2,uint32_t addr);//RedePinなし、２ポート
    ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,PinName TX2,PinName RX2,uint32_t addr);//RedePinあり、１ポート＋RedePinなし、１ポート
    ~ScrpSlave();
    void setTimeout(int);
    void addCMD(uint8_t cmd, bool (*proc)(int rx_data,int& tx_data));
    int send(uint8_t id,uint8_t cmd,int16_t tx_data);
    int send2(uint8_t id,uint8_t cmd,int16_t tx_data);
private:
    DigitalOut *rede_;
    Serial *serial_[2];
    FlashIAP *flash_;
    uint8_t mode_;
    uint8_t my_id_;
    uint32_t address_;
    int timeout_;
    bool (*procs_[256])(int rx_data, int& tx_data);
    int sending(int,uint8_t,uint8_t,int16_t);
    void changeID(uint8_t);
    void check(int port);
    void init(PinName,PinName);
    void port1();
    void port2();
};

#endif /* SCRP_SLAVE_H */