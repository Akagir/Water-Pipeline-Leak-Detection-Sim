#ifndef __FLORA_WATERLEAKLORAAPP_H_
#define __FLORA_WATERLEAKLORAAPP_H_

#include <omnetpp.h>

#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"
#include "inet/common/Units.h"

#include "LoRaAppPacket_m.h"
#include "LoRa/LoRaMacControlInfo_m.h"
#include "LoRa/LoRaRadio.h"

using namespace omnetpp;
using namespace inet;

namespace flora {

class WaterLeakLoRaApp : public cSimpleModule, public ILifecycle
{
  protected:
    enum SelfMessageKind {
        SEND_PRESSURE_REPORT = 100,
        SEND_LEAK_ALARM = 101
    };

    cMessage *sendReportTimer = nullptr;
    cMessage *sendLeakTimer = nullptr;

    int reportsSent = 0;
    int alarmsSent = 0;
    int receivedADRCommands = 0;

    simtime_t reportInterval;
    double leakLambda;

    cOutVector pressureVector;
    cOutVector leakInterArrivalVector;
    cOutVector alarmSentVector;
    cOutVector sfVector;
    cOutVector tpVector;

    LoRaRadio *loRaRadio = nullptr;

    bool evaluateADRinNode = false;
    int ADR_ACK_CNT = 0;
    int ADR_ACK_LIMIT = 64;
    int ADR_ACK_DELAY = 32;
    bool sendNextPacketWithADRACKReq = false;

    cMessage *retryTxTimer = nullptr;

    bool pendingPressureReport = false;
    bool pendingLeakAlarm = false;

    simtime_t nextTxAllowedTime = 0;
    simtime_t txGuardTime;

    simsignal_t LoRa_AppPacketSent;

  protected:
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessage(cMessage *msg) override;
    virtual bool handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback) override;

    void scheduleNextLeak();
    void sendPressureReport();
    void sendLeakAlarm();
    void sendWaterPacket(bool isLeakAlarm);
    void handleMessageFromLowerLayer(cMessage *msg);

    void trySendPendingPacket();
    void scheduleRetryTx();

    void setSF(int SF);
    int getSF();

    void setTP(int TP);
    double getTP();

    void setCR(int CR);
    int getCR();

    void setCF(units::values::Hz CF);
    units::values::Hz getCF();

    void setBW(units::values::Hz BW);
    units::values::Hz getBW();

    void increaseSFIfPossible();

  public:
    WaterLeakLoRaApp() {}
    virtual ~WaterLeakLoRaApp();
};

}

#endif
