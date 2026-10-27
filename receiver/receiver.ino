#include <SPI.h>
#include <math.h>
#include <RadioLib.h>
#include "config.h"

SX1278 radio = new Module(RADIO_CS,RADIO_DIO0,RADIO_RST,RADIOLIB_NC);
static volatile bool packetReady=false;
static uint32_t rxCount=0,lastSequence=0,missingSequences=0;
static uint32_t seen[64]={0};static uint8_t seenWrite=0;
static uint32_t activeEvent=0,activeTriggerMs=0,berRunActive=0;
static uint16_t berExpected=0,berReceived=0;
static char serialBuffer[300];static uint16_t serialLength=0;
static uint32_t linkWindowFrames=0,linkWindowMissing=0,linkWindowDuplicates=0;
static uint32_t linkWindowTxLogical=0,linkWindowTxFailed=0,linkWindowTxAttempts=0,linkWindowTxRetries=0;
static uint32_t lastTxLogical=0,lastTxFailed=0,lastTxAttempts=0,lastTxRetries=0;
static bool txCounterBaselineValid=false;
static float linkWindowRssi=0,linkWindowSnr=0;
static uint32_t lastV1PacketMs=0,nextLinkCommandId=60000,lastLinkAttemptMs=0;
static uint8_t activeLinkLevel=0,badLinkWindows=0,goodLinkWindows=0;
static uint8_t lastReportedTxPower=RADIO_TX_DBM;
static bool linkStateInitialized=false,linkOfflineReported=false,linkCommandBusy=false;

bool waitCommandReply(const String &command,uint32_t id,String &response);

void onPacketReceived(){packetReady=true;}

String field(const String &packet,uint8_t wanted){
  int start=0;
  for(uint8_t i=0;i<wanted;i++){int comma=packet.indexOf(',',start);if(comma<0)return String("");start=comma+1;}
  int end=packet.indexOf(',',start);return end<0?packet.substring(start):packet.substring(start,end);
}
uint32_t packetSequence(const String &packet){return field(packet,2).toInt();}
bool duplicateSequence(uint32_t seq){if(!seq)return false;for(uint8_t i=0;i<64;i++)if(seen[i]==seq)return true;seen[seenWrite++%64]=seq;return false;}
uint32_t xorshift32(uint32_t &x){x^=x<<13;x^=x>>17;x^=x<<5;return x;}

const char *linkName(uint8_t level){switch(level){case 0:return "GOOD";case 1:return "FAIR";case 2:return "WEAK";default:return "CRITICAL";}}
uint8_t chunkForLevel(uint8_t level){return level==0?LINK_CHUNK_GOOD:level==1?LINK_CHUNK_FAIR:level==2?LINK_CHUNK_WEAK:LINK_CHUNK_CRITICAL;}
uint8_t powerForLevel(uint8_t level){uint8_t requested=level==0?LINK_POWER_GOOD_DBM:level==1?LINK_POWER_FAIR_DBM:level==2?LINK_POWER_WEAK_DBM:LINK_POWER_CRITICAL_DBM;return requested>RADIO_TX_DBM?RADIO_TX_DBM:requested;}

void observeLink(float rssi,float snr,bool duplicate,uint32_t missingDelta){
  linkWindowFrames++;linkWindowRssi+=rssi;linkWindowSnr+=snr;
  if(duplicate)linkWindowDuplicates++;
  linkWindowMissing+=missingDelta;
}

void resetLinkWindow(){
  linkWindowFrames=linkWindowMissing=linkWindowDuplicates=0;
  linkWindowTxLogical=linkWindowTxFailed=linkWindowTxAttempts=linkWindowTxRetries=0;
  linkWindowRssi=linkWindowSnr=0;
}

bool sendAdaptiveCommand(uint8_t level,float avgRssi,float avgSnr,float lossPct,float retryPct){
  if(linkCommandBusy)return false;
  linkCommandBusy=true;lastLinkAttemptMs=millis();
  uint8_t chunk=chunkForLevel(level),power=powerForLevel(level);uint32_t id=++nextLinkCommandId;
  String command=String("LINK,")+id+","+level+","+power+","+chunk,response;
  bool ok=waitCommandReply(command,id,response);
  bool applied=ok&&response.indexOf(",LINK_APPLIED,")>=0;
  if(applied){
    activeLinkLevel=level;lastReportedTxPower=power;linkStateInitialized=true;
    Serial.printf("LINK_STATE,%u,%s,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,adaptive\n",level,linkName(level),avgRssi,avgSnr,lossPct,retryPct,power,chunk,level<2?1:0);
    if(level>=2&&berRunActive){Serial.printf("BER_ABORT,LINK_WEAK,%u\n",level);berRunActive=0;}
  }else Serial.printf("LINK_CONTROL_FAIL,%u,%s\n",level,ok?response.c_str():"NO_ACK");
  linkCommandBusy=false;return applied;
}

void evaluateLinkWindow(){
  if(linkWindowFrames<LINK_WINDOW_FRAMES)return;
  float n=linkWindowFrames,avgRssi=linkWindowRssi/n,avgSnr=linkWindowSnr/n;
  float rxLoss=100.0f*(linkWindowMissing+linkWindowDuplicates)/fmaxf(1.0f,n+linkWindowMissing+linkWindowDuplicates);
  float txFail=100.0f*linkWindowTxFailed/fmaxf(1.0f,linkWindowTxLogical);
  float retries=100.0f*linkWindowTxRetries/fmaxf(1.0f,linkWindowTxAttempts);
  float loss=rxLoss>txFail?rxLoss:txFail;
  float classifiedRssi=avgRssi;
  if(linkStateInitialized&&lastReportedTxPower>powerForLevel(activeLinkLevel))classifiedRssi-=lastReportedTxPower-powerForLevel(activeLinkLevel);
  uint8_t target;
  if(classifiedRssi>=LINK_GOOD_RSSI_DBM&&avgSnr>=LINK_GOOD_SNR_DB&&loss<=LINK_GOOD_LOSS_PCT&&retries<=LINK_GOOD_RETRY_PCT)target=0;
  else if(classifiedRssi>=LINK_FAIR_RSSI_DBM&&avgSnr>=LINK_FAIR_SNR_DB&&loss<=LINK_FAIR_LOSS_PCT&&retries<=LINK_FAIR_RETRY_PCT)target=1;
  else if(classifiedRssi>=LINK_WEAK_RSSI_DBM&&avgSnr>=LINK_WEAK_SNR_DB&&loss<=LINK_WEAK_LOSS_PCT&&retries<=LINK_WEAK_RETRY_PCT)target=2;
  else target=3;
  uint32_t now=millis();bool change=false;uint8_t requested=activeLinkLevel;
  if(!linkStateInitialized){requested=target;change=true;}
  else if(target>activeLinkLevel){goodLinkWindows=0;if(++badLinkWindows>=LINK_BAD_WINDOWS_TO_DOWNGRADE){requested=activeLinkLevel+1;badLinkWindows=0;change=true;}}
  else if(target<activeLinkLevel){badLinkWindows=0;if(++goodLinkWindows>=LINK_GOOD_WINDOWS_TO_UPGRADE){requested=activeLinkLevel-1;goodLinkWindows=0;change=true;}}
  else{
    badLinkWindows=0;
    if(lastReportedTxPower>powerForLevel(activeLinkLevel)){
      if(++goodLinkWindows>=LINK_GOOD_WINDOWS_TO_UPGRADE){requested=activeLinkLevel;goodLinkWindows=0;change=true;}
    }else goodLinkWindows=0;
  }
  if(change&&(now-lastLinkAttemptMs>1000))sendAdaptiveCommand(requested,avgRssi,avgSnr,loss,retries);
  else if(linkStateInitialized)Serial.printf("LINK_STATE,%u,%s,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,monitor\n",activeLinkLevel,linkName(activeLinkLevel),avgRssi,avgSnr,loss,retries,powerForLevel(activeLinkLevel),chunkForLevel(activeLinkLevel),activeLinkLevel<2?1:0);
  linkWindowFrames=linkWindowMissing=linkWindowDuplicates=0;linkWindowRssi=linkWindowSnr=0;
  linkWindowTxLogical=linkWindowTxFailed=linkWindowTxAttempts=linkWindowTxRetries=0;
}

void observeTransmitterCounters(const String &packet){
  uint32_t logical=field(packet,10).toInt(),failed=field(packet,12).toInt(),attempts=field(packet,13).toInt(),retries=field(packet,14).toInt();
  String reportedPower=field(packet,17);if(reportedPower.length())lastReportedTxPower=reportedPower.toInt();
  if(!txCounterBaselineValid){lastTxLogical=logical;lastTxFailed=failed;lastTxAttempts=attempts;lastTxRetries=retries;txCounterBaselineValid=true;return;}
  if(logical>=lastTxLogical)linkWindowTxLogical+=logical-lastTxLogical;else linkWindowTxLogical+=logical;
  if(failed>=lastTxFailed)linkWindowTxFailed+=failed-lastTxFailed;else linkWindowTxFailed+=failed;
  if(attempts>=lastTxAttempts)linkWindowTxAttempts+=attempts-lastTxAttempts;else linkWindowTxAttempts+=attempts;
  if(retries>=lastTxRetries)linkWindowTxRetries+=retries-lastTxRetries;else linkWindowTxRetries+=retries;
  lastTxLogical=logical;lastTxFailed=failed;lastTxAttempts=attempts;lastTxRetries=retries;
}

void emitEventSamples(const String &packet){
  uint32_t event=field(packet,3).toInt();uint16_t chunk=field(packet,4).toInt();
  uint16_t chunks=field(packet,5).toInt();uint16_t first=field(packet,6).toInt();
  String body=field(packet,7);uint16_t local=0;
  while(body.length()){
    int sep=body.indexOf(';');String record=sep<0?body:body.substring(0,sep);body=sep<0?String(""):body.substring(sep+1);
    float offset=0,ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
    if(sscanf(record.c_str(),"%f:%f:%f:%f:%f:%f:%f",&offset,&ax,&ay,&az,&gx,&gy,&gz)==7)
      Serial.printf("EVENT_SAMPLE,%lu,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n",(unsigned long)event,(unsigned)(first+local),offset,ax,ay,az,gx,gy,gz);
    local++;
  }
  Serial.printf("EVENT_CHUNK,%lu,%u,%u,%u\n",(unsigned long)event,chunk,chunks,local);
}

void emitBerSample(const String &packet,float rssi,float snr){
  uint32_t run=field(packet,3).toInt(),index=field(packet,4).toInt();String hex=field(packet,5);
  if(hex.length()!=BER_PAYLOAD_BYTES*2)return;
  uint32_t state=run^(index*0x9e3779b9UL)^0xa5c39e17UL;uint16_t errors=0;
  for(uint8_t i=0;i<BER_PAYLOAD_BYTES;i++){
    uint8_t expected=xorshift32(state)&0xff;
    auto nibble=[](char c)->int{if(c>='0'&&c<='9')return c-'0';c|=32;if(c>='a'&&c<='f')return c-'a'+10;return -1;};
    int hi=nibble(hex[i*2]),lo=nibble(hex[i*2+1]);uint8_t actual=(hi<0||lo<0)?0:(uint8_t)((hi<<4)|lo);
    errors+=__builtin_popcount((unsigned)(expected^actual));
  }
  if(run==berRunActive)berReceived++;
  Serial.printf("BER_SAMPLE,%lu,%lu,%lu,%u,%u,%.1f,%.1f\n",(unsigned long)run,(unsigned long)index,(unsigned long)packetSequence(packet),BER_PAYLOAD_BITS,errors,rssi,snr);
}

void transmitAck(uint32_t seq){String ack=String("ACK,")+seq;Serial.printf("RADIO_TX,%s\n",ack.c_str());radio.transmit(ack);}

void processRadioPacket(const String &packet,int16_t radioState){
  if(!packet.startsWith("V1,")){Serial.printf("PACKET,%s\n",packet.c_str());return;}
  float rssi=radio.getRSSI(),snr=radio.getSNR();uint32_t seq=packetSequence(packet);
  bool duplicate=duplicateSequence(seq);rxCount++;uint32_t missingDelta=0;
  bool resumed=linkOfflineReported;
  if(resumed){
    // Sequence numbers and TX retry counters can advance unseen during an
    // outage. Reset measurement baselines without counting that old gap as
    // current loss, while retaining the last applied adaptation level.
    resetLinkWindow();txCounterBaselineValid=false;badLinkWindows=goodLinkWindows=0;
    if(seq)lastSequence=seq;
  }else if(seq&&!duplicate){if(lastSequence&&seq>lastSequence+1){missingDelta=seq-lastSequence-1;missingSequences+=missingDelta;}if(seq>lastSequence)lastSequence=seq;}
  lastV1PacketMs=millis();linkOfflineReported=false;
  if(resumed&&linkStateInitialized)Serial.printf("LINK_STATE,%u,%s,%.1f,%.1f,0,0,%u,%u,%u,reconnected\n",activeLinkLevel,linkName(activeLinkLevel),radio.getRSSI(),radio.getSNR(),powerForLevel(activeLinkLevel),chunkForLevel(activeLinkLevel),activeLinkLevel<2?1:0);
  Serial.printf("RADIO,%lu,%.1f,%.1f,%lu,%u,%lu\n",(unsigned long)millis(),rssi,snr,(unsigned long)seq,duplicate?1:0,(unsigned long)missingSequences);
  String type=field(packet,1);
  if(!duplicate){
    if(type=="HB"){
      observeTransmitterCounters(packet);
      Serial.printf("HEARTBEAT,%s\n",packet.substring(6).c_str());
    }else if(type=="EVT"){
      activeEvent=field(packet,3).toInt();activeTriggerMs=field(packet,4).toInt();
      Serial.printf("EVENT_META,%s\n",packet.substring(7).c_str());
    }else if(type=="SMP")emitEventSamples(packet);
    else if(type=="TEST")emitBerSample(packet,rssi,snr);
    else if(type=="BEND"){
      uint32_t run=field(packet,3).toInt();uint16_t expected=field(packet,4).toInt();
      Serial.printf("BER_DONE,%lu,%u,%u,%u\n",(unsigned long)run,expected,berReceived,expected>berReceived?expected-berReceived:0);
      if(run==berRunActive)berRunActive=0;
    }else Serial.printf("PACKET,%s\n",packet.c_str());
  }
  observeLink(rssi,snr,duplicate,missingDelta);
  if(type!="TEST"&&seq)transmitAck(seq);
}

bool waitCommandReply(const String &command,uint32_t id,String &response){
  String expected=String("RSP,")+id+",";
  for(uint8_t attempt=0;attempt<COMMAND_ATTEMPTS;attempt++){
    Serial.printf("RADIO_TX,%s,attempt=%u\n",command.c_str(),attempt+1);
    // RadioLib's String overload takes a mutable String&, while command is
    // const because it is also reused for retries and diagnostics.
    String txCommand=command;
    int16_t tx=radio.transmit(txCommand);if(tx!=RADIOLIB_ERR_NONE){radio.startReceive();continue;}
    int16_t state=radio.receive(response,0,ACK_TIMEOUT_MS);packetReady=false;
    if(state==RADIOLIB_ERR_NONE&&response.startsWith(expected)){radio.startReceive();return true;}
    if(state==RADIOLIB_ERR_NONE){Serial.printf("RADIO_RX,%s\n",response.c_str());processRadioPacket(response,state);}
    radio.startReceive();
  }
  return false;
}

void handleHostCommand(const String &command){
  int comma=command.indexOf(',');if(comma<0)return;int next=command.indexOf(',',comma+1);
  uint32_t id=(next<0?command.substring(comma+1):command.substring(comma+1,next)).toInt();
  if(command.startsWith("BER,")){
    int c1=command.indexOf(',',comma+1),c2=command.indexOf(',',c1+1);
    uint32_t run=command.substring(c1+1,c2).toInt();uint16_t count=command.substring(c2+1).toInt();
    if(berRunActive!=run){berRunActive=run;berExpected=count;berReceived=0;}
  }
  if(command.startsWith("BER_STOP,"))berRunActive=0;
  String response;bool ok=waitCommandReply(command,id,response);
  if(ok){Serial.printf("RADIO_RX,%s\n",response.c_str());Serial.printf("CMD_ACK,%s\n",response.substring(4).c_str());}
  else Serial.printf("CMD_ERROR,%lu,NO_RADIO_ACK\n",(unsigned long)id);
}

void serviceSerial(){
  while(Serial.available()){
    char c=Serial.read();if(c=='\r')continue;
    if(c!='\n'){if(serialLength<sizeof(serialBuffer)-1)serialBuffer[serialLength++]=c;continue;}
    serialBuffer[serialLength]=0;serialLength=0;String command(serialBuffer);
    if(command.startsWith("SET,")||command.startsWith("BER,")||command.startsWith("BER_STOP,"))handleHostCommand(command);
  }
}

void setup(){
  Serial.begin(SERIAL_BAUD);delay(200);SPI.begin(RADIO_SCK,RADIO_MISO,RADIO_MOSI,RADIO_CS);
  int16_t state=radio.begin(RADIO_FREQ_MHZ,RADIO_BW_KHZ,RADIO_SF,RADIO_CR,RADIO_SYNC_WORD,RADIO_TX_DBM,RADIO_PREAMBLE);
  if(state!=RADIOLIB_ERR_NONE){Serial.printf("FATAL,RADIO_INIT,%d\n",state);while(true)delay(1000);}
  radio.setCRC(true);radio.setPacketReceivedAction(onPacketReceived);state=radio.startReceive();
  if(state!=RADIOLIB_ERR_NONE){Serial.printf("FATAL,RADIO_RX_START,%d\n",state);while(true)delay(1000);}
  lastV1PacketMs=millis();Serial.println("READY,RX,radio=SX1278,baud=115200,link_adaptation=enabled");
}

void loop(){
  if(packetReady){packetReady=false;String packet;int16_t state=radio.readData(packet);if(state==RADIOLIB_ERR_NONE)processRadioPacket(packet,state);else Serial.printf("RADIO_ERROR,RX_READ,%d\n",state);int16_t rx=radio.startReceive();if(rx!=RADIOLIB_ERR_NONE)Serial.printf("RADIO_ERROR,RX_RESTART,%d\n",rx);}
  evaluateLinkWindow();
  if(!linkOfflineReported&&millis()-lastV1PacketMs>=LINK_STALE_MS){Serial.println("LINK_STATE,4,OFFLINE,0,0,100,0,1,0,no_packets");linkOfflineReported=true;resetLinkWindow();txCounterBaselineValid=false;badLinkWindows=goodLinkWindows=0;}
  serviceSerial();
}
