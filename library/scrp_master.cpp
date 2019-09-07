#include "scrp_master.hpp"

#define STX 0x41
#define DMY 0xff

ScrpMaster::ScrpMaster(PinName TX1,PinName RX1){
    mode_ = 0;
    init(TX1,RX1);
}

ScrpMaster::ScrpMaster(PinName TX1,PinName RX1,PinName REDE1){
    mode_ = 1;
    rede_ = new DigitalOut(REDE1);
    init(TX1,RX1);
}

ScrpMaster::ScrpMaster(PinName TX1,PinName RX1,PinName TX2,PinName RX2){
    mode_ = 2;
    serial_[1] = new Serial(TX2,RX2,115200);
    init(TX1,RX1);
}

ScrpMaster::ScrpMaster(PinName TX1,PinName RX1,PinName REDE1,PinName TX2,PinName RX2){
    mode_ = 3;
    rede_ = new DigitalOut(REDE1);
    serial_[1] = new Serial(TX2,RX2,115200);
    init(TX1,RX1);
}

void ScrpMaster::init(PinName TX,PinName RX){
    timeout_ = 100;
    serial_[0] = new Serial(TX,RX,115200);
}

void ScrpMaster::setTimeout(int time){
    timeout_ = time;
}

int ScrpMaster::send(uint8_t id,uint8_t cmd,int16_t tx_data){
    return sending(0,id,cmd,tx_data);
}

int ScrpMaster::send2(uint8_t id,uint8_t cmd,int16_t tx_data){
    if(mode_ < 2){
        return -1;
    }
    return sending(1,id,cmd,tx_data);
}

int ScrpMaster::sending(int port,uint8_t id,uint8_t cmd,int16_t tx_data){
    uint8_t tx_dataL = tx_data;
    uint8_t tx_dataH = tx_data >> 8;
    uint8_t tx_sum = id + cmd + tx_dataL + tx_dataH;

    const uint8_t data[] = {DMY, STX, id, cmd, tx_dataL, tx_dataH, tx_sum, DMY};
    if(!serial_[port]->writeable()){
        return -1;
    }
    if(mode_%2 == 1 && id == 0){
        rede_->write(1);
    }
    for(int i = 0;i<8;i++){
        serial_[port]->putc(data[i]);
    }
    while(!serial_[port]->writeable());
    if(mode_%2 == 1 && id == 0){
        rede_->write(0);
    }/*
            
    int i = 0;
    bool received = false;
    bool stxflag = false;
    uint8_t rx[5]={},sum = 0;
    Timer out;
    out.start();
    while(out.read_ms() < timeout_ && !received){
        while(serial_[port]->readable()){
            if(serial_[port]->getc() == STX && !stxflag){
                stxflag = true;
                continue;
            }
            if(stxflag){
                rx[i] = serial_[port]->getc();
                sum += rx[i++];
            }
            if(i > 4){
                if(sum == rx[4]){
                    received = true;
                }
                break;
            }
        }
    }
    out.stop();
    if(!received){
        return -1;
    }
    return (rx[2] + (rx[3] << 8));*/
    return 0;
}

ScrpMaster::~ScrpMaster(){
    delete serial_[0];
    if(mode_%2 == 1){
        delete rede_;
    }
    if(mode_ >= 2){
        delete serial_[1];
    }
}