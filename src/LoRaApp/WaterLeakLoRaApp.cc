#include "WaterLeakLoRaApp.h"

#include "inet/common/packet/Packet.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/TimeTag_m.h"
#include "inet/mobility/static/StationaryMobility.h"

#include "../LoRa/LoRaTagInfo_m.h"

namespace flora {

Define_Module(WaterLeakLoRaApp);

WaterLeakLoRaApp::~WaterLeakLoRaApp()
{
    cancelAndDelete(sendReportTimer);
    cancelAndDelete(sendLeakTimer);
    cancelAndDelete(retryTxTimer);
}

void WaterLeakLoRaApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;

        if (!isOperational) {
            throw cRuntimeError("WaterLeakLoRaApp does not support starting in node DOWN state");
        }

        reportInterval = par("reportInterval");
        leakLambda = par("leakLambda");

        txGuardTime = par("txGuardTime");

        sendReportTimer = new cMessage("sendPressureReport");
        sendReportTimer->setKind(SEND_PRESSURE_REPORT);

        sendLeakTimer = new cMessage("sendLeakAlarm");
        sendLeakTimer->setKind(SEND_LEAK_ALARM);

        retryTxTimer = new cMessage("retryTxTimer");

        if (leakLambda <= 0) {
            throw cRuntimeError("leakLambda must be greater than 0");
        }

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        pressureVector.setName("pressureValue");
        leakInterArrivalVector.setName("leakInterArrivalTime");
        alarmSentVector.setName("alarmSent");
        sfVector.setName("SF Vector");
        tpVector.setName("TP Vector");

        loRaRadio = check_and_cast<LoRaRadio *>(
            getParentModule()->getSubmodule("LoRaNic")->getSubmodule("radio")
        );

        loRaRadio->loRaTP = par("initialLoRaTP").doubleValue();
        loRaRadio->loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaRadio->loRaSF = par("initialLoRaSF");
        loRaRadio->loRaBW = units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaRadio->loRaCR = par("initialLoRaCR");
        loRaRadio->loRaUseHeader = par("initialUseHeader");

        evaluateADRinNode = par("evaluateADRinNode");

        sendReportTimer = new cMessage("sendPressureReport");
        sendReportTimer->setKind(SEND_PRESSURE_REPORT);

        sendLeakTimer = new cMessage("sendLeakAlarm");
        sendLeakTimer->setKind(SEND_LEAK_ALARM);

        simtime_t firstReport = par("timeToFirstReport");
        scheduleAt(simTime() + firstReport, sendReportTimer);

        scheduleNextLeak();
    }
}

void WaterLeakLoRaApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        if (msg == sendReportTimer) {
            pendingPressureReport = true;

            scheduleAt(simTime() + reportInterval, sendReportTimer);

            trySendPendingPacket();
        }
        else if (msg == sendLeakTimer) {
            pendingLeakAlarm = true;

            scheduleNextLeak();

            trySendPendingPacket();
        }
        else if (msg == retryTxTimer) {
            trySendPendingPacket();
        }
    }
    else {
        handleMessageFromLowerLayer(msg);
        delete msg;
    }
}

void WaterLeakLoRaApp::scheduleNextLeak()
{
    // Poisson leak process:
    // exponential inter-arrival time with mean 1 / lambda.
    simtime_t interArrivalTime = exponential(1.0 / leakLambda);

    leakInterArrivalVector.record(interArrivalTime.dbl());

    scheduleAt(simTime() + interArrivalTime, sendLeakTimer);
}

void WaterLeakLoRaApp::trySendPendingPacket()
{
    if (simTime() < nextTxAllowedTime) {
        scheduleRetryTx();
        return;
    }

    bool sentSomething = false;

    // Leak alarms get priority over normal pressure reports
    if (pendingLeakAlarm) {
        pendingLeakAlarm = false;
        sendLeakAlarm();
        sentSomething = true;
    }
    else if (pendingPressureReport) {
        pendingPressureReport = false;
        sendPressureReport();
        sentSomething = true;
    }

    // If another packet is still waiting, retry later
    if (sentSomething && (pendingLeakAlarm || pendingPressureReport)) {
        scheduleRetryTx();
    }
}

void WaterLeakLoRaApp::scheduleRetryTx()
{
    simtime_t retryTime = nextTxAllowedTime + uniform(0, 1);

    if (retryTime <= simTime()) {
        retryTime = simTime() + txGuardTime + uniform(0, 1);
    }

    if (retryTxTimer->isScheduled()) {
        cancelEvent(retryTxTimer);
    }

    scheduleAt(retryTime, retryTxTimer);
}

void WaterLeakLoRaApp::sendPressureReport()
{
    reportsSent++;

    double pressure = uniform(45.0, 55.0);
    pressureVector.record(pressure);

    EV << "LoRa node[" << getParentModule()->getIndex()
       << "] sends pressure report at time "
       << simTime()
       << ", pressure=" << pressure << endl;

    sendWaterPacket(false);
}

void WaterLeakLoRaApp::sendLeakAlarm()
{
    alarmsSent++;
    alarmSentVector.record(alarmsSent);

    EV << "LoRa node[" << getParentModule()->getIndex()
       << "] sends LEAK_ALARM at time "
       << simTime() << endl;

    bubble("Leak alarm sent");

    sendWaterPacket(true);
}

void WaterLeakLoRaApp::sendWaterPacket(bool isLeakAlarm)
{
    Packet *pkt = new Packet(isLeakAlarm ? "leakAlarm" : "pressureReport");
    pkt->setKind(DATA);

    auto payload = makeShared<LoRaAppPacket>();
    payload->setMsgType(DATA);
    payload->setChunkLength(B(par("dataSize").intValue()));

    /*
     * Encoding:
     *  - normal pressure report: pressure value around 45–55
     *  - leak alarm: negative marker value
     *
     * This allows you to see leak packets in debugging.
     * FLoRa's default NetworkServerApp records total received packets,
     * not separate alarm/report counters.
     */
    if (isLeakAlarm) {
        payload->setSampleMeasurement(-9999);
    }
    else {
        payload->setSampleMeasurement((int)uniform(45, 55));
    }

    if (evaluateADRinNode && sendNextPacketWithADRACKReq) {
        auto opt = payload->getOptions();
        opt.setADRACKReq(true);
        payload->setOptions(opt);
        sendNextPacketWithADRACKReq = false;
    }

    auto loraTag = pkt->addTagIfAbsent<LoRaTag>();
    loraTag->setBandwidth(getBW());
    loraTag->setCenterFrequency(getCF());
    loraTag->setSpreadFactor(getSF());
    loraTag->setCodeRendundance(getCR());
    loraTag->setPower(mW(math::dBmW2mW(getTP())));

    sfVector.record(getSF());
    tpVector.record(getTP());

    pkt->insertAtBack(payload);

    send(pkt, "socketOut");

    nextTxAllowedTime = simTime() + txGuardTime;

    emit(LoRa_AppPacketSent, getSF());

    if (evaluateADRinNode) {
        ADR_ACK_CNT++;

        if (ADR_ACK_CNT == ADR_ACK_LIMIT) {
            sendNextPacketWithADRACKReq = true;
        }

        if (ADR_ACK_CNT >= ADR_ACK_LIMIT + ADR_ACK_DELAY) {
            ADR_ACK_CNT = 0;
            increaseSFIfPossible();
        }
    }
}

void WaterLeakLoRaApp::handleMessageFromLowerLayer(cMessage *msg)
{
    auto pkt = check_and_cast<Packet *>(msg);
    const auto &packet = pkt->peekAtFront<LoRaAppPacket>();

    if (simTime() >= getSimulation()->getWarmupPeriod()) {
        receivedADRCommands++;
    }

    if (evaluateADRinNode) {
        ADR_ACK_CNT = 0;

        if (packet->getMsgType() == TXCONFIG) {
            if (packet->getOptions().getLoRaTP() != -1) {
                setTP(packet->getOptions().getLoRaTP());
            }

            if (packet->getOptions().getLoRaSF() != -1) {
                setSF(packet->getOptions().getLoRaSF());
            }

            EV << "ADR update received. New TP=" << getTP()
               << ", New SF=" << getSF() << endl;
        }
    }
}

bool WaterLeakLoRaApp::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();
    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

void WaterLeakLoRaApp::finish()
{
    recordScalar("reportsSent", reportsSent);
    recordScalar("alarmsSent", alarmsSent);
    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("finalTP", getTP());
    recordScalar("finalSF", getSF());
}

void WaterLeakLoRaApp::increaseSFIfPossible()
{
    if (getSF() < 12) {
        setSF(getSF() + 1);
    }
}

void WaterLeakLoRaApp::setSF(int SF)
{
    loRaRadio->loRaSF = SF;
}

int WaterLeakLoRaApp::getSF()
{
    return loRaRadio->loRaSF;
}

void WaterLeakLoRaApp::setTP(int TP)
{
    loRaRadio->loRaTP = TP;
}

double WaterLeakLoRaApp::getTP()
{
    return loRaRadio->loRaTP;
}

void WaterLeakLoRaApp::setCF(units::values::Hz CF)
{
    loRaRadio->loRaCF = CF;
}

units::values::Hz WaterLeakLoRaApp::getCF()
{
    return loRaRadio->loRaCF;
}

void WaterLeakLoRaApp::setBW(units::values::Hz BW)
{
    loRaRadio->loRaBW = BW;
}

units::values::Hz WaterLeakLoRaApp::getBW()
{
    return loRaRadio->loRaBW;
}

void WaterLeakLoRaApp::setCR(int CR)
{
    loRaRadio->loRaCR = CR;
}

int WaterLeakLoRaApp::getCR()
{
    return loRaRadio->loRaCR;
}

}
