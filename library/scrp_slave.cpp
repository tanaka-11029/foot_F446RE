#include "scrp_slave.hpp"

#define STX 0x41
#define DMY 0xff

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,uint32_t addr):address_(addr){
    mode_ = 0;
    init(TX1,RX1);
}

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,uint32_t addr):address_(addr){
    mode_ = 1;
    rede_ = new DigitalOut(REDE1);
    init(TX1,RX1);
}

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,PinName TX2,PinName RX2,uint32_t addr):address_(addr){
    mode_ = 2;
    serial_[1] = new Serial(TX2,RX2,115200);
    serial_[1]->attach(callback(this,&ScrpSlave::port2),Serial::RxIrq);
    init(TX1,RX1);
}

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,PinName TX2,PinName RX2,uint32_t addr):address_(addr){
    mode_ = 3;
    rede_ = new DigitalOut(REDE1);
    serial_[1] = new Serial(TX2,RX2,115200);
    serial_[1]->attach(callback(this,&ScrpSlave::port2),Serial::RxIrq);
    init(TX1,RX1);
}

void ScrpSlave::init(PinName TX,PinName RX){
    timeout_ = 100;
    serial_[0] = new Serial(TX,RX,115200);
    serial_[0]->attach(callback(this,&ScrpSlave::port1),Serial::RxIrq);
    flash_ = new FlashIAP;
    if(flash_->init()==0){
        if(flash_->read(&my_id_,address_,1) != 0){
            send(222,222,222);
            my_id_ = 10;
        }
    }else{
        send(111,111,111);
        my_id_ = 10;
    }
    for(int i = 1;i<255;++i){
        procs_[i] = 0;
    }
}

void ScrpSlave::port1(){
    check(0);
}

void ScrpSlave::port2(){
    check(1);
}

void ScrpSlave::addCMD(uint8_t cmd, bool (*proc)(int rx_data, int& tx_data)){
    if(cmd == 0 || cmd == 254 || cmd == 253)return;
    procs_[cmd] = proc;
}

void ScrpSlave::setTimeout(int time){
    timeout_ = time;
}

void ScrpSlave::changeID(uint8_t id){
    flash_->erase(address_,flash_->get_sector_size(address_));
    flash_->program(&id,address_,1);
}

int ScrpSlave::send(uint8_t id,uint8_t cmd,int16_t tx_data){
    return sending(0,id,cmd,tx_data);
}

int ScrpSlave::send2(uint8_t id,uint8_t cmd,int16_t tx_data){
    if(mode_ < 2)return -1;
    return sending(1,id,cmd,tx_data);
}

int ScrpSlave::sending(int port,uint8_t id,uint8_t cmd,int16_t tx_data){
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
    }
            
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
            if(i > 4){/*
                uint8_t sum = 0;
                for(int j = 0;j<4;j++){
                    sum += rx[j];
                }*/
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
    return (rx[2] + (rx[3] << 8));
}

void ScrpSlave::check(int port){
    uint8_t rx_cmd;
    int16_t rx_data;
    bool received = false;
    bool broadcast = false;
    while(serial_[port]->readable()){
        if(serial_[port]->getc() != STX)continue;
        uint8_t rx_id = serial_[port]->getc();
        uint8_t tmp_rx_cmd = serial_[port]->getc();
        uint8_t tmp_rx_dataL = serial_[port]->getc();
        uint8_t tmp_rx_dataH = serial_[port]->getc();
        uint8_t rx_sum = serial_[port]->getc();
        
        uint8_t sum = rx_id + tmp_rx_cmd + tmp_rx_dataL + tmp_rx_dataH;
        if(sum != rx_sum){
            continue;
        }
        
        if(rx_id == 255){
            broadcast = true;
        }else if(my_id_ == rx_id){
            broadcast = false;
        }else{
            break;
        }
        
        rx_cmd = tmp_rx_cmd;
        rx_data = tmp_rx_dataL + ((int16_t)tmp_rx_dataH << 8);
        received = true;
    }
    if(!received){
        return;
    }
    int tx_data = rx_data;
    if(rx_cmd == 0){
        tx_data = rx_data;
    }else if(rx_cmd == 254){
        uint8_t new_id = rx_data;
        my_id_ = new_id;
        changeID(new_id);
    }else if(rx_cmd == 253){
        tx_data = my_id_;
        rx_cmd = 250;
        broadcast = false;
    }else if(procs_[rx_cmd] == 0 || !procs_[rx_cmd](rx_data,tx_data)){
        return;
    }
    if(broadcast){
        return;
    }
    
    uint8_t tx_dataL = tx_data;
    uint8_t tx_dataH = tx_data >> 8;
    uint8_t tx_sum = my_id_ + rx_cmd + tx_dataL + tx_dataH;
    
    const uint8_t data[] = {DMY, STX, my_id_, rx_cmd, tx_dataL, tx_dataH, tx_sum, DMY};
    if(!serial_[port]->writeable()){
        return;
    }
    if(mode_%2 == 1 && port == 0){
        rede_->write(1);
    }
    for(int i = 0;i<8;i++){
        serial_[port]->putc(data[i]);
    }
    while(!serial_[port]->writeable());
    if(mode_%2 == 1 && port == 0){
        rede_->write(0);
    }    
    return;
}

ScrpSlave::~ScrpSlave(){
    delete serial_[0];
    delete flash_;
    if(mode_%2 == 1){
        delete rede_;
    }
    if(mode_ >= 2){
        delete serial_[1];
    }
}