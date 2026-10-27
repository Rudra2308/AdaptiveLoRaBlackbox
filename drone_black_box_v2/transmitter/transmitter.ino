#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Preferences.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RadioLib.h>
#include "config.h"

// Sensor acquisition runs in Arduino loop() on core 1. All radio operations
// run in radioTask() on core 0, so ACK timeouts never stop event sampling.
Adafruit_MPU6050 imu;
SX1278 radio = new Module(RADIO_CS, RADIO_DIO0, RADIO_RST, RADIOLIB_NC);
Preferences preferences;

struct Sample { uint32_t ms; float ax, ay, az, gx, gy, gz; };
struct EventPacket {
  uint32_t id, trigger_ms, config_revision;
  uint16_t count, start_index, total_count;
  char stage;
  char reason[40];
  float peak_accel, peak_gyro, risk, cumulative_risk, pre_seconds, post_seconds;
  Sample samples[EVENT_CAPACITY];
};
struct LiveStatus {
  uint32_t samples, config_revision, event_queue_drops;
  float accel_magnitude, gyro_magnitude;
};

static Sample ringBuffer[EVENT_CAPACITY];
static uint16_t ringCount=0, ringWrite=0;
static QueueHandle_t eventQueue=nullptr;
static portMUX_TYPE stateMux=portMUX_INITIALIZER_UNLOCKED;
static volatile bool radioPacketReady=false;
static DetectionConfig liveConfig=detection;
static LiveStatus liveStatus={0,1,0,0,0};
static uint32_t gyroBiasMs=0;
static float gyroBiasX=0,gyroBiasY=0,gyroBiasZ=0;
static float referenceX=0,referenceY=0,referenceZ=1;
static uint32_t nextSequence=1, nextEventId=0, lastHeartbeatMs=0;
static uint32_t txLogical=0,txAcked=0,txFailed=0,txAttempts=0,txRetries=0;
static uint32_t ackLatencyTotalMs=0,ackCount=0;
static uint32_t berRun=0,berCount=0,berIndex=0,lastBerMs=0;
static bool berActive=false,berEndPending=false;
static uint8_t adaptiveLinkLevel=0,eventChunkSamples=ADAPTIVE_DEFAULT_CHUNK_SAMPLES;
static int8_t adaptivePowerDbm=ADAPTIVE_MAX_TX_DBM;
static uint8_t consecutiveTxFailures=0;

static uint32_t recentEventMs[16];
static uint8_t recentEventCount=0;
static uint32_t lastEventMs=0,tiltSinceMs=0,eventTriggerMs=0,lastRiskMs=0;
static float cumulativeRisk=0;
static bool capturing=false;
static uint16_t postSamplesLeft=0,eventSampleCount=0;
static uint16_t preSampleCount=0,postSampleCount=0,postWrite=0;
static Sample postSamples[(EVENT_CAPACITY/2)+1];
static DetectionConfig captureConfig;
static uint32_t captureConfigRevision=1;
static float eventPeakAccel=0,eventPeakGyro=0,eventAccelSq=0,eventRisk=0;
static char eventReason[40]="";
static uint32_t nextSampleUs=0,lastSampleUs=0;

void onRadioPacket() { radioPacketReady=true; }
void fail(const char *message) { Serial.println(message); while(true) delay(1000); }

void addSample(const Sample &sample) {
  ringBuffer[ringWrite]=sample;
  ringWrite=(ringWrite+1)%EVENT_CAPACITY;
  if(ringCount<EVENT_CAPACITY) ringCount++;
}

bool validConfig(const DetectionConfig &c) {
  return c.impact_mps2>=0.1f&&c.impact_mps2<=78.45f&&
    c.gyro_rad_s>=0.05f&&c.gyro_rad_s<=8.73f&&
    c.tilt_deg>=1&&c.tilt_deg<=180&&c.tilt_duration_ms>=50&&c.tilt_duration_ms<=60000&&
    c.repeated_window_ms>=100&&c.repeated_window_ms<=600000&&c.repeated_count>=2&&c.repeated_count<=16&&
    c.cooldown_ms<=60000&&c.pre_seconds>=0.5f&&c.pre_seconds<=2&&
    c.post_seconds>=0.5f&&c.post_seconds<=2&&c.pre_seconds+c.post_seconds<=4&&
    c.impact_weight>=0&&c.rotation_weight>=0&&c.repeated_weight>=0&&
    c.impact_weight+c.rotation_weight+c.repeated_weight>0&&c.decay_per_hour>=0&&c.decay_per_hour<=1;
}

void sendCommandReply(uint32_t id,const char *result,uint32_t value=0) {
  String reply=String("RSP,")+id+","+result;
  if(value) reply+=String(",")+value;
  Serial.printf("RADIO_TX,%s\n",reply.c_str());
  radio.transmit(reply);
  radio.startReceive();
}

void handleRadioCommand(const String &command) {
  if(command.startsWith("SET,")) {
    unsigned long id=0,revision=0,tiltMs=0,windowMs=0,cooldownMs=0;
    unsigned int repeated=0;
    DetectionConfig c;
    if(sscanf(command.c_str(),"SET,%lu,%lu,%f,%f,%f,%lu,%lu,%u,%lu,%f,%f,%f,%f,%f,%f",
      &id,&revision,&c.impact_mps2,&c.gyro_rad_s,&c.tilt_deg,&tiltMs,&windowMs,&repeated,
      &cooldownMs,&c.pre_seconds,&c.post_seconds,&c.impact_weight,&c.rotation_weight,
      &c.repeated_weight,&c.decay_per_hour)!=15) { sendCommandReply(id,"SET_REJECT"); return; }
    c.tilt_duration_ms=tiltMs;c.repeated_window_ms=windowMs;c.repeated_count=repeated;c.cooldown_ms=cooldownMs;
    if(!validConfig(c)) { sendCommandReply(id,"SET_REJECT"); return; }
    portENTER_CRITICAL(&stateMux);liveConfig=c;liveStatus.config_revision=revision;portEXIT_CRITICAL(&stateMux);
    preferences.putFloat("impact",c.impact_mps2);preferences.putFloat("gyro",c.gyro_rad_s);
    preferences.putFloat("tilt",c.tilt_deg);preferences.putUInt("tilt_ms",tiltMs);
    preferences.putUInt("rep_win",windowMs);preferences.putUChar("rep_n",repeated);
    preferences.putUInt("cool_ms",cooldownMs);preferences.putFloat("pre",c.pre_seconds);
    preferences.putFloat("post",c.post_seconds);preferences.putFloat("wi",c.impact_weight);
    preferences.putFloat("wg",c.rotation_weight);preferences.putFloat("wr",c.repeated_weight);
    preferences.putFloat("decay",c.decay_per_hour);preferences.putUInt("revision",revision);
    sendCommandReply(id,"SET_OK",revision);return;
  }
  if(command.startsWith("LINK,")) {
    unsigned long id=0;unsigned int level=0,power=0,chunk=0;
    if(sscanf(command.c_str(),"LINK,%lu,%u,%u,%u",&id,&level,&power,&chunk)!=4||
       level>3||power<ADAPTIVE_MIN_TX_DBM||power>ADAPTIVE_MAX_TX_DBM||
       chunk<ADAPTIVE_MIN_CHUNK_SAMPLES||chunk>ADAPTIVE_DEFAULT_CHUNK_SAMPLES) {
      sendCommandReply(id,"LINK_REJECT");return;
    }
    int16_t state=radio.setOutputPower((int8_t)power);
    if(state!=RADIOLIB_ERR_NONE){Serial.printf("LINK_APPLY_ERROR,%d\n",state);sendCommandReply(id,"LINK_REJECT");return;}
    adaptiveLinkLevel=level;adaptivePowerDbm=power;eventChunkSamples=chunk;consecutiveTxFailures=0;
    if(level>=ADAPTIVE_WEAK_LEVEL&&(berActive||berEndPending)){
      berActive=false;berEndPending=false;
      Serial.printf("BER_ABORT,LINK_LEVEL,%u\n",level);
    }
    Serial.printf("LINK_APPLIED,%u,%d,%u\n",adaptiveLinkLevel,adaptivePowerDbm,eventChunkSamples);
    sendCommandReply(id,"LINK_APPLIED",power);return;
  }
  if(command.startsWith("BER,")) {
    unsigned long id=0,run=0,count=0;
    if(sscanf(command.c_str(),"BER,%lu,%lu,%lu",&id,&run,&count)!=3||count<1||count>10000) { sendCommandReply(id,"BER_REJECT");return; }
    if(adaptiveLinkLevel>=ADAPTIVE_WEAK_LEVEL){sendCommandReply(id,"BER_LINK_BLOCKED",adaptiveLinkLevel);return;}
    if(!berActive||berRun!=run){berRun=run;berCount=count;berIndex=0;berEndPending=false;lastBerMs=0;}
    berActive=true;
    sendCommandReply(id,"BER_STARTED",run);return;
  }
  if(command.startsWith("BER_STOP,")) {
    unsigned long id=0;sscanf(command.c_str(),"BER_STOP,%lu",&id);
    berActive=false;berEndPending=false;sendCommandReply(id,"BER_STOPPED");return;
  }
}

bool sendReliable(const String &frame,uint32_t sequence) {
  txLogical++;
  uint32_t started=millis();
  for(uint8_t attempt=0;attempt<=MAX_RETRIES;attempt++) {
    txAttempts++;if(attempt)txRetries++;
    Serial.printf("RADIO_TX,%s,attempt=%u\n",frame.c_str(),attempt+1);
    String txFrame=frame;
    int16_t state=radio.transmit(txFrame);
    if(state!=RADIOLIB_ERR_NONE) continue;
    String reply;state=radio.receive(reply,0,ACK_TIMEOUT_MS);radioPacketReady=false;
    if(state==RADIOLIB_ERR_NONE) {
      Serial.printf("RADIO_RX,%s\n",reply.c_str());
      if(reply==String("ACK,")+sequence) {
        txAcked++;ackCount++;ackLatencyTotalMs+=millis()-started;
        consecutiveTxFailures=0;
        radio.startReceive();return true;
      }
      if(reply.startsWith("SET,")||reply.startsWith("BER,")||reply.startsWith("BER_STOP,")||reply.startsWith("LINK,")) handleRadioCommand(reply);
    }
  }
  txFailed++;
  if(consecutiveTxFailures<255)consecutiveTxFailures++;
  // If ACKs disappear, raise power locally so the transmitter can restore
  // the link even though receiver feedback cannot currently get through.
  if(consecutiveTxFailures>=2&&adaptivePowerDbm<ADAPTIVE_MAX_TX_DBM){
    int8_t nextPower=adaptivePowerDbm+2;
    if(nextPower>ADAPTIVE_MAX_TX_DBM)nextPower=ADAPTIVE_MAX_TX_DBM;
    int16_t powerState=radio.setOutputPower(nextPower);
    if(powerState==RADIOLIB_ERR_NONE){adaptivePowerDbm=nextPower;Serial.printf("LINK_FALLBACK_POWER,%d\n",adaptivePowerDbm);}
    consecutiveTxFailures=0;
  }
  radio.startReceive();return false;
}

void sendDiagnostic(const String &frame) {
  Serial.printf("RADIO_TX,%s,one_shot=1\n",frame.c_str());
  String txFrame=frame;
  int16_t state=radio.transmit(txFrame);
  if(state!=RADIOLIB_ERR_NONE) Serial.printf("RADIO_ERROR,BER_TX,%d\n",state);
  radio.startReceive();
}

void sendEvent(EventPacket *event) {
  uint32_t seq=nextSequence++;
  String meta=String("V1,EVT,")+seq+","+event->id+","+event->trigger_ms+","+event->reason+","+
    String(event->peak_accel,2)+","+String(event->peak_gyro,3)+","+String(event->risk,1)+","+
    String(event->cumulative_risk,1)+","+event->total_count+","+event->config_revision+","+
    String(event->pre_seconds,1)+","+String(event->post_seconds,1)+","+
    String(gyroBiasX,5)+","+String(gyroBiasY,5)+","+String(gyroBiasZ,5)+","+
    (event->stage=='P'?"PRE":"POST")+","+event->start_index;
  if(!sendReliable(meta,seq)) { Serial.printf("EVENT_TX_FAILED,%lu,META\n",(unsigned long)event->id);return; }
  uint8_t samplesPerChunk=eventChunkSamples;
  uint16_t chunks=(event->count+samplesPerChunk-1)/samplesPerChunk;
  for(uint16_t chunk=0;chunk<chunks;chunk++) {
    uint32_t chunkSeq=nextSequence++;
    uint16_t start=chunk*samplesPerChunk;
    uint16_t end=min((uint16_t)(start+samplesPerChunk),event->count);
    String frame=String("V1,SMP,")+chunkSeq+","+event->id+","+chunk+","+chunks+","+(event->start_index+start)+",";
    for(uint16_t i=start;i<end;i++) {
      const Sample &s=event->samples[i];if(i>start)frame+=';';
      frame+=String((int32_t)(s.ms-event->trigger_ms))+":"+String(s.ax,2)+":"+String(s.ay,2)+":"+
        String(s.az,2)+":"+String(s.gx,3)+":"+String(s.gy,3)+":"+String(s.gz,3);
    }
    if(frame.length()>240||!sendReliable(frame,chunkSeq)) { Serial.printf("EVENT_TX_FAILED,%lu,%u\n",(unsigned long)event->id,chunk);return; }
    uint32_t now=millis();
    if(now-lastHeartbeatMs>=HEARTBEAT_INTERVAL_MS) {
      lastHeartbeatMs=now;LiveStatus status;portENTER_CRITICAL(&stateMux);status=liveStatus;portEXIT_CRITICAL(&stateMux);
      uint32_t hbSeq=nextSequence++;String hb=String("V1,HB,")+hbSeq+","+DEVICE_ID+","+now+","+status.config_revision+","+status.samples+","+
        String(status.accel_magnitude,2)+","+String(status.gyro_magnitude,3)+","+status.event_queue_drops+","+
        txLogical+","+txAcked+","+txFailed+","+txAttempts+","+txRetries+","+ackLatencyTotalMs+","+ackCount+","+
        String((int)adaptivePowerDbm)+","+String((unsigned)adaptiveLinkLevel)+","+String((unsigned)eventChunkSamples);
      sendReliable(hb,hbSeq);
    }
  }
}

bool queueEvent(EventPacket *event) {
  if(!event)return false;
  if(xQueueSend(eventQueue,&event,0)==pdTRUE)return true;
  vPortFree(event);
  portENTER_CRITICAL(&stateMux);liveStatus.event_queue_drops++;portEXIT_CRITICAL(&stateMux);
  return false;
}

void radioTask(void *) {
  for(;;) {
    EventPacket *event=nullptr;
    if(xQueueReceive(eventQueue,&event,pdMS_TO_TICKS(5))==pdTRUE&&event) { sendEvent(event);vPortFree(event); }
    uint32_t now=millis();
    if(now-lastHeartbeatMs>=HEARTBEAT_INTERVAL_MS) {
      lastHeartbeatMs=now;LiveStatus s;portENTER_CRITICAL(&stateMux);s=liveStatus;portEXIT_CRITICAL(&stateMux);
      uint32_t seq=nextSequence++;
      String hb=String("V1,HB,")+seq+","+DEVICE_ID+","+now+","+s.config_revision+","+s.samples+","+
        String(s.accel_magnitude,2)+","+String(s.gyro_magnitude,3)+","+s.event_queue_drops+","+
        txLogical+","+txAcked+","+txFailed+","+txAttempts+","+txRetries+","+ackLatencyTotalMs+","+ackCount+","+
        String((int)adaptivePowerDbm)+","+String((unsigned)adaptiveLinkLevel)+","+String((unsigned)eventChunkSamples);
      sendReliable(hb,seq);
    }
    if(berActive&&!berEndPending&&now-lastBerMs>=BER_INTERVAL_MS) {
      lastBerMs=now;uint32_t seq=nextSequence++;
      uint32_t x=berRun^(berIndex*0x9e3779b9UL)^0xa5c39e17UL;
      char hex[BER_PAYLOAD_BYTES*2+1];const char *digits="0123456789ABCDEF";
      for(uint8_t i=0;i<BER_PAYLOAD_BYTES;i++){x^=x<<13;x^=x>>17;x^=x<<5;uint8_t b=x&0xff;hex[2*i]=digits[b>>4];hex[2*i+1]=digits[b&15];}
      hex[sizeof(hex)-1]=0;sendDiagnostic(String("V1,TEST,")+seq+","+berRun+","+berIndex+","+hex);
      berIndex++;if(berIndex>=berCount){berActive=false;berEndPending=true;}
    }
    if(berEndPending&&now-lastBerMs>=BER_INTERVAL_MS) {
      uint32_t seq=nextSequence++;sendReliable(String("V1,BEND,")+seq+","+berRun+","+berCount,seq);berEndPending=false;
    }
    if(radioPacketReady) {
      radioPacketReady=false;String command;int16_t state=radio.readData(command);
      if(state==RADIOLIB_ERR_NONE) { Serial.printf("RADIO_RX,%s\n",command.c_str());handleRadioCommand(command); }
      else radio.startReceive();
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void loadConfig() {
  liveConfig.impact_mps2=preferences.getFloat("impact",detection.impact_mps2);
  liveConfig.gyro_rad_s=preferences.getFloat("gyro",detection.gyro_rad_s);
  liveConfig.tilt_deg=preferences.getFloat("tilt",detection.tilt_deg);
  liveConfig.tilt_duration_ms=preferences.getUInt("tilt_ms",detection.tilt_duration_ms);
  liveConfig.repeated_window_ms=preferences.getUInt("rep_win",detection.repeated_window_ms);
  liveConfig.repeated_count=preferences.getUChar("rep_n",detection.repeated_count);
  liveConfig.cooldown_ms=preferences.getUInt("cool_ms",detection.cooldown_ms);
  liveConfig.pre_seconds=preferences.getFloat("pre",detection.pre_seconds);
  liveConfig.post_seconds=preferences.getFloat("post",detection.post_seconds);
  liveConfig.impact_weight=preferences.getFloat("wi",detection.impact_weight);
  liveConfig.rotation_weight=preferences.getFloat("wg",detection.rotation_weight);
  liveConfig.repeated_weight=preferences.getFloat("wr",detection.repeated_weight);
  liveConfig.decay_per_hour=preferences.getFloat("decay",detection.decay_per_hour);
  liveStatus.config_revision=preferences.getUInt("revision",1);
  if(!validConfig(liveConfig)) liveConfig=detection;
}

void setup() {
  Serial.begin(SERIAL_BAUD);delay(200);
  Wire.begin(I2C_SDA,I2C_SCL);Wire.setClock(400000);
  if(!imu.begin(0x68,&Wire)&&!imu.begin(0x69,&Wire)) fail("FATAL,MPU6050_NOT_FOUND,check_0x68_0x69_and_wiring");
  imu.setAccelerometerRange(MPU6050_RANGE_8_G);imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  sensors_event_t a,g,t;float refX=0,refY=0,refZ=0;
  for(uint16_t i=0;i<200;i++){imu.getEvent(&a,&g,&t);gyroBiasX+=g.gyro.x;gyroBiasY+=g.gyro.y;gyroBiasZ+=g.gyro.z;refX+=a.acceleration.x;refY+=a.acceleration.y;refZ+=a.acceleration.z;delay(5);}
  gyroBiasX/=200;gyroBiasY/=200;gyroBiasZ/=200;float norm=sqrtf(refX*refX+refY*refY+refZ*refZ);
  if(norm>0.1f){referenceX=refX/norm;referenceY=refY/norm;referenceZ=refZ/norm;}
  SPI.begin(RADIO_SCK,RADIO_MISO,RADIO_MOSI,RADIO_CS);
  int16_t state=radio.begin(RADIO_FREQ_MHZ,RADIO_BW_KHZ,RADIO_SF,RADIO_CR,RADIO_SYNC_WORD,RADIO_TX_DBM,RADIO_PREAMBLE);
  if(state!=RADIOLIB_ERR_NONE){Serial.printf("FATAL,RADIO_INIT,%d\n",state);while(true)delay(1000);}
  radio.setCRC(true);radio.setPacketReceivedAction(onRadioPacket);radio.startReceive();
  preferences.begin("cargo",false);loadConfig();
  eventQueue=xQueueCreate(2,sizeof(EventPacket*));if(!eventQueue)fail("FATAL,EVENT_QUEUE_CREATE_FAILED");
  nextEventId=esp_random();nextSampleUs=micros();lastHeartbeatMs=millis();
  xTaskCreatePinnedToCore(radioTask,"radioTask",8192,nullptr,1,nullptr,0);
  Serial.println("READY,TX,radio_task_core0,sampler_core1,rate_hz=50");
  Serial.printf("CAL,gyro_bias=%.5f:%.5f:%.5f,gravity_ref=%.3f:%.3f:%.3f\n",gyroBiasX,gyroBiasY,gyroBiasZ,referenceX,referenceY,referenceZ);
}

void loop() {
  uint32_t nowUs=micros();int32_t wait=(int32_t)(nextSampleUs-nowUs);
  if(wait>0){if(wait>1000)delay(1);else delayMicroseconds(wait);return;}
  if((uint32_t)(nowUs-nextSampleUs)>SAMPLE_PERIOD_US*3) nextSampleUs=nowUs+SAMPLE_PERIOD_US;
  else nextSampleUs+=SAMPLE_PERIOD_US;
  sensors_event_t a,g,t;imu.getEvent(&a,&g,&t);
  Sample s={millis(),a.acceleration.x,a.acceleration.y,a.acceleration.z,g.gyro.x,g.gyro.y,g.gyro.z};
  float amag=sqrtf(s.ax*s.ax+s.ay*s.ay+s.az*s.az);
  float cgx=s.gx-gyroBiasX,cgy=s.gy-gyroBiasY,cgz=s.gz-gyroBiasZ;
  float gmag=sqrtf(cgx*cgx+cgy*cgy+cgz*cgz);
  DetectionConfig c;portENTER_CRITICAL(&stateMux);c=liveConfig;portEXIT_CRITICAL(&stateMux);
  float dot=(s.ax*referenceX+s.ay*referenceY+s.az*referenceZ)/fmaxf(amag,0.1f);dot=fminf(1.0f,fmaxf(-1.0f,dot));
  float tilt=acosf(dot)*57.2957795f;
  if(tilt>=c.tilt_deg){if(!tiltSinceMs)tiltSinceMs=s.ms;}else tiltSinceMs=0;
  bool impact=amag>=c.impact_mps2,rotation=gmag>=c.gyro_rad_s;
  bool tiltHeld=tiltSinceMs&&(s.ms-tiltSinceMs)>=c.tilt_duration_ms;
  bool trigger=impact||rotation||tiltHeld;
  addSample(s);
  portENTER_CRITICAL(&stateMux);liveStatus.samples++;liveStatus.accel_magnitude=amag;liveStatus.gyro_magnitude=gmag;portEXIT_CRITICAL(&stateMux);

  bool newTrigger=trigger&&!capturing&&(s.ms-lastEventMs)>=c.cooldown_ms;
  if(newTrigger){
    uint8_t keep=0;for(uint8_t i=0;i<recentEventCount;i++)if(s.ms-recentEventMs[i]<=c.repeated_window_ms)recentEventMs[keep++]=recentEventMs[i];recentEventCount=keep;if(recentEventCount<16)recentEventMs[recentEventCount++]=s.ms;
    bool repeated=recentEventCount>=c.repeated_count;
    eventReason[0]=0;if(impact)strcat(eventReason,"IMPACT");if(rotation){if(eventReason[0])strcat(eventReason,"+");strcat(eventReason,"ROTATION");}if(tiltHeld){if(eventReason[0])strcat(eventReason,"+");strcat(eventReason,"TILT");}if(repeated){if(eventReason[0])strcat(eventReason,"+");strcat(eventReason,"REPEATED");}
    capturing=true;lastEventMs=s.ms;eventTriggerMs=s.ms;nextEventId++;captureConfig=c;
    portENTER_CRITICAL(&stateMux);captureConfigRevision=liveStatus.config_revision;portEXIT_CRITICAL(&stateMux);
    preSampleCount=min((uint16_t)(ringCount>0?ringCount-1:0),(uint16_t)(SAMPLE_HZ*PRE_SECONDS));
    postSampleCount=(uint16_t)(SAMPLE_HZ*POST_SECONDS)+1;postWrite=0;postSamplesLeft=postSampleCount-1;
    eventPeakAccel=amag;eventPeakGyro=gmag;eventSampleCount=0;eventAccelSq=0;
    postSamples[postWrite++]=s;eventSampleCount++;
    EventPacket *pre=(EventPacket*)pvPortMalloc(sizeof(EventPacket));
    if(pre){pre->id=nextEventId;pre->trigger_ms=eventTriggerMs;pre->config_revision=captureConfigRevision;pre->stage='P';pre->start_index=0;pre->count=preSampleCount;pre->total_count=preSampleCount+postSampleCount;pre->peak_accel=eventPeakAccel;pre->peak_gyro=eventPeakGyro;pre->risk=0;pre->cumulative_risk=cumulativeRisk;pre->pre_seconds=PRE_SECONDS;pre->post_seconds=POST_SECONDS;strlcpy(pre->reason,eventReason,sizeof(pre->reason));uint16_t current=(ringWrite+EVENT_CAPACITY-1)%EVENT_CAPACITY;uint16_t first=(current+EVENT_CAPACITY-preSampleCount)%EVENT_CAPACITY;for(uint16_t i=0;i<preSampleCount;i++)pre->samples[i]=ringBuffer[(first+i)%EVENT_CAPACITY];queueEvent(pre);}
    else {portENTER_CRITICAL(&stateMux);liveStatus.event_queue_drops++;portEXIT_CRITICAL(&stateMux);}
  }
  if(capturing){
    eventPeakAccel=fmaxf(eventPeakAccel,amag);eventPeakGyro=fmaxf(eventPeakGyro,gmag);eventAccelSq+=amag*amag;
    if(s.ms!=eventTriggerMs&&postWrite<postSampleCount){postSamples[postWrite++]=s;eventSampleCount++;if(postSamplesLeft)postSamplesLeft--;}
    if(postSamplesLeft==0){
      const DetectionConfig &ec=captureConfig;
      float risk=(ec.impact_weight*fminf(eventPeakAccel/40.0f,1.0f)+ec.rotation_weight*fminf(eventPeakGyro/8.0f,1.0f)+ec.repeated_weight*fminf(recentEventCount/5.0f,1.0f))*100.0f;
      if(lastRiskMs&&ec.decay_per_hour>0){float hours=(s.ms-lastRiskMs)/3600000.0f;cumulativeRisk*=ec.decay_per_hour>=1?0:expf(logf(1-ec.decay_per_hour)*hours);}lastRiskMs=s.ms;cumulativeRisk=fminf(1000.0f,cumulativeRisk+risk);
      EventPacket *post=(EventPacket*)pvPortMalloc(sizeof(EventPacket));
      if(post){post->id=nextEventId;post->trigger_ms=eventTriggerMs;post->config_revision=captureConfigRevision;post->stage='F';post->start_index=preSampleCount;post->count=postWrite;post->total_count=preSampleCount+postSampleCount;post->peak_accel=eventPeakAccel;post->peak_gyro=eventPeakGyro;post->risk=risk;post->cumulative_risk=cumulativeRisk;post->pre_seconds=PRE_SECONDS;post->post_seconds=POST_SECONDS;strlcpy(post->reason,eventReason,sizeof(post->reason));for(uint16_t i=0;i<postWrite;i++)post->samples[i]=postSamples[i];queueEvent(post);}
      else {portENTER_CRITICAL(&stateMux);liveStatus.event_queue_drops++;portEXIT_CRITICAL(&stateMux);}
      capturing=false;
    }
  }
}
