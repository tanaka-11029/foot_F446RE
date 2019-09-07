#include "rotary_inc.hpp"

RotaryInc::RotaryInc(PinName pinA, PinName pinB,int mode):mode_(mode){
    measur_ = false;
    init(pinA,pinB);
}

RotaryInc::RotaryInc(PinName pinA,PinName pinB,double circumference,int resolution,int mode)
    :mode_(mode),resolution_(resolution),circumference_(circumference){
    measur_ = true;
    init(pinA,pinB);
}

void RotaryInc::init(PinName pinA,PinName pinB){
    reset();
    A_ = new InterruptIn(pinA,PullUp);
    B_ = new InterruptIn(pinB,PullUp);
    A_->rise(callback(this,&RotaryInc::riseA));
        
    if(mode_ == 2){
        A_->fall(callback(this,&RotaryInc::fallA));
    }else if(mode_ == 4){
        A_->fall(callback(this,&RotaryInc::fallA));
        B_->rise(callback(this,&RotaryInc::riseB));
        B_->fall(callback(this,&RotaryInc::fallB));
    }else{
        mode_ = 1;
    }
}

void RotaryInc::zero(){
    timer_.stop();
    timer_.reset();
    start_frag_ = false;
    flag_ = false;
    last_[0] = pulse_;
    speed_ = 0;
    count_ = 0;
    sum_ = 0;
    now_ = 0;
}

void RotaryInc::calcu(){
    if(!start_frag_){
        timer_.start();
        start_frag_ = true;
        last_[0] = pulse_;
        pre_t_[0] = 0;
        count_ = 1;
    }else if(flag_){
        now_ = timer_.read();
        timer_.reset();
        sum_ -= pre_t_[count_];
        pre_t_[count_] = now_;
        sum_ += now_;
        speed_ = (double)(pulse_ - last_[count_]) / sum_;
        last_[count_] = pulse_;
        if(count_ < 19){
            count_++;
        }else{
            count_ = 0;
        }
    }else{
        now_ = timer_.read();
        timer_.reset();
        pre_t_[count_] = now_;
        sum_ += now_;
        speed_ = (double)(pulse_ - last_[0]) / sum_;
        last_[count_] = pulse_;
        count_++;
        if(count_ > 19){
            count_ = 0;
            flag_ = true;
        }
    }
}

void RotaryInc::riseA(){
    B_->read() ? pulse_-- : pulse_++;
    if(measur_){
        calcu();
    }
}

void RotaryInc::fallA(){
    B_->read() ? pulse_++ : pulse_--;
    if(measur_){
        calcu();
    }
}

void RotaryInc::riseB(){
    A_->read() ? pulse_++ : pulse_--;
    if(measur_){
        calcu();
    }
}

void RotaryInc::fallB(){
    A_->read() ? pulse_-- : pulse_++;
    if(measur_){
        calcu();
    }
}

long long RotaryInc::get(){
    return pulse_;
}

double RotaryInc::getSpeed(){
    if(!measur_){
        return 0;
    }
    if(timer_.read_ms() > 150){
        zero();
    }
    return speed_ / resolution_ / mode_ * circumference_;
}

int RotaryInc::diff(){
    int diff = pulse_ - prev_;
    prev_ = pulse_;
    return diff;
}
    
void RotaryInc::reset(){
    pulse_ = 0;
    prev_ = 0;
    if(measur_){
        zero();
    }
}

RotaryInc::~RotaryInc(){
    A_->disable_irq();
    B_->disable_irq();
    delete A_;
    delete B_;
}