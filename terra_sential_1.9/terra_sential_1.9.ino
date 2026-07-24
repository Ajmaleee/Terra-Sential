// TERRA SENTINEL v1.9 — Landslide Detection OS
// ESP32 + SH1106 128x64 (I2C 21/22) + Capacitive Soil (ADC 32)
// Buzzer GPIO25 | UP GPIO15 | SEL GPIO4 | DOWN GPIO5
// Libs: U8g2, ArduinoJson — both via Library Manager
// ESP32 Arduino core 3.x required.
#pragma GCC optimize("Os","ffunction-sections","fdata-sections")

#include <U8g2lib.h>
#include <Wire.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "driver/gpio.h"
#include "esp_sleep.h"

// ── Pins ──────────────────────────────────────
#define PIN_UP    15
#define PIN_SEL    4
#define PIN_DOWN   5
#define PIN_SOIL  32
#define PIN_BUZ   25
#define I2C_SDA   21
#define I2C_SCL   22

// ── Soil calibration ─────────────────────────
#define ADC_DRY  3400
#define ADC_WET  1200
#define ADC_SAMP    8

// ── Timing ───────────────────────────────────
#define T_SENSOR   2000
#define T_DISPLAY   100
#define T_HOLD      500
#define T_BLINK     400
#define T_SLEEP   80000ULL
#define T_IDLE    30000
#define T_WIFI_TO 10000
#define T_WIFI_RT 15000

// ── Firebase RTDB (HTTP / DB Secret) ─────────
#define FB_URL    "https://landslide-detection-syst-8f3b1-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FB_SECRET "Tg4DwMIUtKM0naEIHUutmfB6rdU2NYRuMWdYCuFw"

// ── AP ───────────────────────────────────────
#define AP_SSID "TerraSetup"
#define AP_PASS "terra1234"

// ── Defaults ─────────────────────────────────
#define DEF_WATCH   55
#define DEF_WARN    70
#define DEF_CRIT    85
#define DEF_PUSH  10000

// ── LEDC (Core 3.x) ──────────────────────────
#define BUZ_CH  0
#define BUZ_RES 10

#define FW_VER   "1.9.0"
#define DEV_NAME "TERRA SENTINEL"

// ─────────────────────────────────────────────
//  DISPLAY
// ─────────────────────────────────────────────
U8G2_SH1106_128X64_NONAME_F_HW_I2C
    disp(U8G2_R0, U8X8_PIN_NONE, I2C_SCL, I2C_SDA);

// ─────────────────────────────────────────────
//  TYPES
// ─────────────────────────────────────────────
enum class AL   : uint8_t { OK=0,WATCH=1,WARN=2,CRIT=3 };
enum class View : uint8_t { DASH=0,SENS=1,ALERTS=2,NET=3,CFG=4,ABOUT=5,VIEW_COUNT };
enum class NS   : uint8_t { DOWN=0,CING=1,UP=2,FB=3,ERR=4,AP=5 };

struct Soil { uint16_t raw=0; uint8_t pct=0; AL al=AL::OK; };
// sf=short-fired(one-shot on release), lf=long-fired(one-shot at hold threshold),
// lh=long-held(level, true while physically held after threshold)
struct Btn  { bool on=false,sf=false,lf=false,lh=false; uint32_t t=0; };
struct Cfg  { char ssid[64]="",pass[64]=""; uint8_t tw=DEF_WATCH,wr=DEF_WARN,tc=DEF_CRIT; uint32_t pms=DEF_PUSH; };
struct Net  { NS st=NS::DOWN; uint32_t lpush=0,lwifi=0,lpt=0,pc=0; char err[48]=""; uint8_t cli=0; };
struct App  { View cur=View::DASH; uint8_t mc=0; bool mopen=false,aack=false,apOn=false; uint32_t bt=0,la=0; };

// ─────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────
static Soil soil; static Cfg cfg; static App app; static Net net;
static Btn  bU,bS,bD;
static AL   prevAL=AL::OK;
static uint32_t tSens=0,tDisp=0,tBlink=0;
static bool blink=false;

// ─────────────────────────────────────────────
//  BUZZER  (Core 3.x: ledcAttachChannel / ledcChangeFrequency / ledcWrite(PIN,...))
// ─────────────────────────────────────────────
namespace Buz {
    void off()            { ledcWrite(PIN_BUZ,0); }
    void freq(uint32_t f) { ledcChangeFrequency(PIN_BUZ,f,BUZ_RES); ledcWrite(PIN_BUZ,512); }
    void tn(uint32_t f,uint32_t ms){ freq(f); delay(ms); off(); }
    void rt(uint32_t ms)            { off(); delay(ms); }
    void click()  { freq(1800); delay(12); off(); }
    void mopen()  { tn(600,40); rt(15); tn(900,40); }
    void mclose() { tn(900,40); rt(15); tn(600,40); }
    void mnav()   { freq(1200); delay(8); off(); }
    void boot()   { tn(523,80); rt(20); tn(659,80); rt(20); tn(784,120); }
    void awatch() { tn(880,80); rt(60); tn(880,80); }
    void awarn()  { tn(880,90); rt(40); tn(1047,90); rt(40); tn(1245,140); }
    void acrit()  {
        for(uint8_t r=0;r<2;r++){
            tn(1397,70);rt(30);tn(1397,70);rt(30);tn(1397,70);rt(60);
            for(uint16_t f=1400;f>=900;f-=50){ freq(f); delay(8); }
            off(); if(r==0)rt(80);
        }
    }
    void aack() { tn(784,80);rt(20);tn(659,80);rt(20);tn(523,120); }
    void begin(){ ledcAttachChannel(PIN_BUZ,2000,BUZ_RES,BUZ_CH); off(); }
}

// ─────────────────────────────────────────────
//  BUTTONS
// ─────────────────────────────────────────────
namespace Btns {
    void poll(uint8_t pin, Btn &b){
        bool h=(digitalRead(pin)==LOW); uint32_t n=millis();
        if(h&&!b.on){  b.on=true; b.lf=b.lh=b.sf=false; b.t=n; }
        else if(!h&&b.on){ b.on=false; b.lh=false; if(!b.lf)b.sf=true; b.lf=false; }
        else if(h&&b.on&&!b.lf&&(n-b.t)>=T_HOLD){ b.lf=true; b.lh=true; }
    }
    bool cs(Btn &b){ if(b.sf){b.sf=false;return true;}return false; }
    bool cl(Btn &b){ if(b.lf){b.lf=false;return true;}return false; }
}

// ─────────────────────────────────────────────
//  CONFIG — NVS
// ─────────────────────────────────────────────
namespace Cfg_ {
    Preferences p;
    void load(){
        p.begin("ts",true);
        p.getString("s",cfg.ssid,64); p.getString("p",cfg.pass,64);
        cfg.tw=p.getUChar("tw",DEF_WATCH); cfg.wr=p.getUChar("wr",DEF_WARN);
        cfg.tc=p.getUChar("tc",DEF_CRIT);  cfg.pms=p.getULong("pm",DEF_PUSH);
        p.end();
    }
    void save(){
        p.begin("ts",false);
        p.putString("s",cfg.ssid); p.putString("p",cfg.pass);
        p.putUChar("tw",cfg.tw);   p.putUChar("wr",cfg.wr);
        p.putUChar("tc",cfg.tc);   p.putULong("pm",cfg.pms);
        p.end();
    }
}

// ─────────────────────────────────────────────
//  ALERTS
// ─────────────────────────────────────────────
namespace Alerts {
    const char* str(AL a){
        switch(a){ case AL::WATCH:return"WATCH"; case AL::WARN:return"WARNING"; case AL::CRIT:return"CRITICAL"; default:return"NORMAL"; }
    }
    bool active(){ return soil.al!=AL::OK&&!app.aack; }
    void update(){
        if(app.aack&&soil.al==AL::OK) app.aack=false;
        if(soil.al!=prevAL&&soil.al>prevAL){
            switch(soil.al){ case AL::WATCH:Buz::awatch();break; case AL::WARN:Buz::awarn();break; case AL::CRIT:Buz::acrit();break; default:break; }
        }
        prevAL=soil.al;
    }
}

// ─────────────────────────────────────────────
//  SENSORS
// ─────────────────────────────────────────────
namespace Sens {
    void read(){
        uint32_t a=0;
        for(uint8_t i=0;i<ADC_SAMP;i++){ a+=analogRead(PIN_SOIL); delayMicroseconds(500); }
        soil.raw=(uint16_t)(a/ADC_SAMP);
        int32_t p=map((int32_t)soil.raw,ADC_DRY,ADC_WET,0,100);
        soil.pct=(uint8_t)constrain(p,0,100);
        soil.al=soil.pct>=cfg.tc?AL::CRIT:soil.pct>=cfg.wr?AL::WARN:soil.pct>=cfg.tw?AL::WATCH:AL::OK;
    }
}

// ─────────────────────────────────────────────
//  LIGHT SLEEP  (display + I2C stay powered)
// ─────────────────────────────────────────────
namespace Slp {
    void cfg_wake(){
        gpio_wakeup_enable((gpio_num_t)PIN_UP,  GPIO_INTR_LOW_LEVEL);
        gpio_wakeup_enable((gpio_num_t)PIN_SEL, GPIO_INTR_LOW_LEVEL);
        gpio_wakeup_enable((gpio_num_t)PIN_DOWN,GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_gpio_wakeup();
    }
    void tick(uint64_t us){ esp_sleep_enable_timer_wakeup(us); esp_light_sleep_start(); }
}

// ─────────────────────────────────────────────
//  AP CONFIG PORTAL
//  Fixes vs v1.8:
//  1. The ENTIRE page is now assembled into one String and sent with a
//     single srv.send(200,"text/html", page) call. No more sendContent_P
//     fragments — WebServer computes a real Content-Length up front, so
//     the browser knows exactly how many bytes to expect and the page can
//     no longer be cut off mid-stream (weak AP RSSI, browser prefetch,
//     captive-portal probes hitting "/" mid-transfer, etc.)
//  2. Values are substituted via String::replace() on placeholder tokens
//     kept in the PROGMEM template — same F() flash-storage trick as
//     before, just no longer chunked out piece by piece.
//  3. softAP() with explicit channel + settle delay retained.
//  4. DNSServer wildcard → 192.168.4.1 (captive portal) retained.
//  5. Full neumorphic ("soft UI") visual redesign — single background
//     tone, raised/inset shadow pairs instead of borders, no gradients.
// ─────────────────────────────────────────────
namespace AP {
    WebServer srv(80);
    DNSServer dns;

    // Single-piece PROGMEM template. %TOKENS% are replaced at request time.
    static const char PAGE_TMPL[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset=UTF-8>
<meta name=viewport content='width=device-width,initial-scale=1'>
<title>Terra Config</title><style>
*{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#e6e9ef;
  --text:#3a4356;
  --sub:#8a93a6;
  --accent:#5b8def;
  --shadow-d:#b9c0cf;
  --shadow-l:#ffffff;
}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--text);
  min-height:100vh;display:flex;align-items:center;justify-content:center;padding:20px}
.c{background:var(--bg);border-radius:28px;padding:28px 26px;width:100%;max-width:420px;
  box-shadow:12px 12px 24px var(--shadow-d),-12px -12px 24px var(--shadow-l)}
h1{font-size:17px;font-weight:700;color:var(--text);margin-bottom:6px;display:flex;align-items:center;gap:8px}
.tag{font-size:11px;color:var(--sub);margin-bottom:20px;letter-spacing:.3px}
h2{font-size:10px;letter-spacing:1.8px;text-transform:uppercase;color:var(--sub);
  margin:20px 0 10px;font-weight:700}
label{display:block;font-size:12px;color:var(--sub);margin-bottom:6px;font-weight:600}
.field{background:var(--bg);border-radius:14px;margin-bottom:14px;
  box-shadow:inset 5px 5px 10px var(--shadow-d),inset -5px -5px 10px var(--shadow-l)}
input[type=text],input[type=password],input[type=number]{
  width:100%;padding:11px 14px;background:transparent;border:none;
  color:var(--text);font-size:14px;outline:none;font-family:inherit}
.row{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px}
.row .field{margin-bottom:0}
button{width:100%;padding:14px;background:var(--bg);color:var(--accent);
  font-size:14px;font-weight:700;border:none;border-radius:16px;letter-spacing:.3px;
  cursor:pointer;margin-top:22px;
  box-shadow:8px 8px 16px var(--shadow-d),-8px -8px 16px var(--shadow-l);
  transition:box-shadow .12s ease}
button:active{box-shadow:inset 5px 5px 10px var(--shadow-d),inset -5px -5px 10px var(--shadow-l)}
#ok{display:none;color:#3fa66a;font-size:13px;text-align:center;margin-top:14px;font-weight:600}
.hint{font-size:10px;color:var(--sub);text-align:right;margin-top:-8px;margin-bottom:14px}
</style></head><body><div class=c>
<h1>&#127757; Terra Sentinel</h1>
<div class=tag>Firmware v%FWVER% &middot; Setup Portal</div>

<h2>WiFi Credentials</h2>
<label>Network SSID</label>
<div class=field><input type=text id=s name=s placeholder='e.g. MyHomeWiFi' value='%SSID%'></div>
<label>Password</label>
<div class=field><input type=password id=p name=p placeholder='Leave blank to keep current'></div>

<h2>Alert Thresholds (%)</h2>
<div class=row>
  <div><label>Watch</label><div class=field><input type=number id=tw name=tw min=1 max=99 value=%TW%></div></div>
  <div><label>Warning</label><div class=field><input type=number id=wr name=wr min=1 max=99 value=%WR%></div></div>
  <div><label>Critical</label><div class=field><input type=number id=tc name=tc min=1 max=99 value=%TC%></div></div>
</div>

<h2>Firebase Push Interval</h2>
<div class=field><input type=number id=pm name=pm min=5 max=3600 value=%PM%></div>
<div class=hint>seconds</div>

<button onclick="
var f=new URLSearchParams();
['s','p','tw','wr','tc','pm'].forEach(function(k){
var e=document.getElementById(k);if(e)f.append(k,e.value)});
fetch('/save',{method:'POST',body:f}).then(function(r){
if(r.ok)document.getElementById('ok').style.display='block'})">Save &amp; Restart</button>
<div id=ok>&#10003; Saved — restarting...</div>
</div></body></html>)HTML";

    void handleRoot(){
        String page; page.reserve(4200);
        page = FPSTR(PAGE_TMPL);
        char nb[8];

        page.replace("%FWVER%", FW_VER);
        page.replace("%SSID%",  cfg.ssid);

        snprintf(nb,8,"%d",cfg.tw);        page.replace("%TW%",nb);
        snprintf(nb,8,"%d",cfg.wr);        page.replace("%WR%",nb);
        snprintf(nb,8,"%d",cfg.tc);        page.replace("%TC%",nb);
        snprintf(nb,8,"%lu",cfg.pms/1000); page.replace("%PM%",nb);

        // One shot — real Content-Length, cannot be truncated mid-stream.
        srv.send(200,"text/html",page);
    }
    void handleSave(){
        if(srv.hasArg("s"))  srv.arg("s").toCharArray(cfg.ssid,64);
        if(srv.hasArg("p")&&srv.arg("p").length()>0) srv.arg("p").toCharArray(cfg.pass,64);
        if(srv.hasArg("tw")) cfg.tw =(uint8_t)constrain(srv.arg("tw").toInt(),1,99);
        if(srv.hasArg("wr")) cfg.wr =(uint8_t)constrain(srv.arg("wr").toInt(),1,99);
        if(srv.hasArg("tc")) cfg.tc =(uint8_t)constrain(srv.arg("tc").toInt(),1,99);
        if(srv.hasArg("pm")) cfg.pms=(uint32_t)constrain(srv.arg("pm").toInt(),5,3600)*1000UL;
        Cfg_::save();
        srv.send(200,"text/plain","ok");
        delay(300); ESP.restart();
    }
    void handleRedir(){
        srv.sendHeader("Location","http://192.168.4.1/",true);
        srv.send(302,"text/plain","");
    }
    void begin(){
        WiFi.disconnect(true); delay(100);
        WiFi.mode(WIFI_AP);    delay(100);
        WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
        delay(200); // let AP fully init before DNS
        dns.start(53,"*",WiFi.softAPIP());
        srv.on("/",     HTTP_GET,  handleRoot);
        srv.on("/save", HTTP_POST, handleSave);
        srv.onNotFound(handleRedir);
        srv.begin();
        net.st=NS::AP; app.apOn=true;
        Serial.printf("[AP] %s  IP=%s\n",AP_SSID,WiFi.softAPIP().toString().c_str());
    }
    void stop(){
        dns.stop(); srv.stop();
        WiFi.softAPdisconnect(true); delay(100);
        WiFi.mode(WIFI_STA);
        net.st=NS::DOWN; app.apOn=false;
    }
    void handle(){ dns.processNextRequest(); srv.handleClient(); net.cli=WiFi.softAPgetStationNum(); }
}

// ─────────────────────────────────────────────
//  FIREBASE — HTTP PATCH/POST, no SDK
// ─────────────────────────────────────────────
namespace FB {
    const char* stStr(){
        switch(net.st){ case NS::DOWN:return"No WiFi"; case NS::CING:return"Connecting";
            case NS::UP:return"WiFi OK"; case NS::FB:return"Firebase OK";
            case NS::ERR:return"FB Error"; case NS::AP:return"AP Mode"; default:return"?"; }
    }
    void push(){
        if(WiFi.status()!=WL_CONNECTED) return;
        WiFiClientSecure sc; sc.setInsecure();
        HTTPClient http;
        JsonDocument doc;
        doc["pct"]=soil.pct; doc["raw"]=soil.raw;
        doc["al"]=Alerts::str(soil.al); doc["up"]=(uint32_t)(millis()/1000); doc["fw"]=FW_VER;
        String pl; serializeJson(doc,pl);
        String url=String(FB_URL)+"/readings/latest.json?auth=" FB_SECRET;
        if(http.begin(sc,url)){
            http.addHeader("Content-Type","application/json");
            int code=http.PATCH(pl);
            if(code==200){ net.lpt=millis(); net.pc++; net.err[0]='\0'; net.st=NS::FB; }
            else { snprintf(net.err,48,"PATCH %d",code); net.st=NS::ERR; }
            http.end();
        }
        if(net.st==NS::FB){
            url=String(FB_URL)+"/readings/history.json?auth=" FB_SECRET;
            if(http.begin(sc,url)){ http.addHeader("Content-Type","application/json"); http.POST(pl); http.end(); }
        }
    }
    void update(){
        if(app.apOn) return;
        if(!cfg.ssid[0]){ net.st=NS::DOWN; return; }
        uint32_t n=millis();
        if(WiFi.status()!=WL_CONNECTED){
            if(net.st==NS::CING) return;
            if((n-net.lwifi)>=T_WIFI_RT){ net.lwifi=n; net.st=NS::CING; WiFi.begin(cfg.ssid,cfg.pass); }
            return;
        }
        if(net.st==NS::CING||net.st==NS::DOWN) net.st=NS::UP;
        if((n-net.lpush)>=cfg.pms){ net.lpush=n; push(); }
    }
    void begin(){
        if(!cfg.ssid[0]){ net.st=NS::DOWN; return; }
        WiFi.mode(WIFI_STA); WiFi.begin(cfg.ssid,cfg.pass);
        net.st=NS::CING; net.lwifi=millis();
        uint32_t t=millis();
        while(WiFi.status()!=WL_CONNECTED&&(millis()-t)<T_WIFI_TO) delay(100);
        net.st=(WiFi.status()==WL_CONNECTED)?NS::UP:NS::DOWN;
    }
}

// ─────────────────────────────────────────────
//  DRAW PRIMITIVES
//  Layout (px):
//    0–9   status bar   (divider at y=9)
//    10–54 body content (max text baseline = 54 for 5x7 font)
//    55    hint divider
//    56–63 hint text    (baseline = 63)
// ─────────────────────────────────────────────
#define HDIV  9   // status bar bottom divider
#define BDIV 55   // hint bar top divider
#define HTY  63   // hint text baseline

namespace Draw {
    void statusBar(){
        disp.setFont(u8g2_font_4x6_tr);
        disp.drawHLine(0,HDIV,128);
        disp.drawStr(1,8,DEV_NAME);
        uint32_t up=(millis()-app.bt)/1000;
        char b[8]; snprintf(b,8,"%02lu:%02lu",up/3600,(up%3600)/60);
        disp.drawStr(104,8,b);
        if(Alerts::active()&&blink) disp.drawStr(60,8,"!");
    }
    void mbar(uint8_t x,uint8_t y,uint8_t w,uint8_t h,uint8_t p){
        disp.drawFrame(x,y,w,h);
        uint8_t f=(uint8_t)((uint16_t)p*(w-2)/100);
        if(f) disp.drawBox(x+1,y+1,f,h-2);
    }
    void badge(uint8_t x,uint8_t y,AL a){
        if(a==AL::OK) return;
        disp.setFont(u8g2_font_5x7_tr);
        const char*s=Alerts::str(a); uint8_t tw=disp.getStrWidth(s);
        disp.drawRFrame(x,y,tw+4,9,2);
        if(blink||a!=AL::CRIT) disp.drawStr(x+2,y+7,s);
    }
    void divider(uint8_t y){ disp.drawHLine(0,y,128); }
    // hint() always draws at fixed bottom zone — never overlaps body
    void hint(const char*s){ disp.drawHLine(0,BDIV,128); disp.setFont(u8g2_font_4x6_tr); disp.drawStr(1,HTY,s); }
}

// ─────────────────────────────────────────────
//  VIEWS
//  Body baselines: 20 (title), 31, 40, 49  — all ≤ 54 (safe above BDIV=55)
// ─────────────────────────────────────────────
namespace Views {

    void dash(){
        Draw::statusBar();
        disp.setFont(u8g2_font_logisoso16_tr);
        char b[4]; snprintf(b,4,"%3d",soil.pct); disp.drawStr(4,35,b);
        disp.setFont(u8g2_font_5x7_tr);
        disp.drawStr(54,35,"%");
        disp.drawStr(4,45,"SOIL MOISTURE");
        Draw::mbar(4,47,80,8,soil.pct);
        auto tick=[](uint8_t p){ disp.drawVLine(4+1+(uint8_t)((uint16_t)p*78/100),46,10); };
        tick(cfg.tw); tick(cfg.wr); tick(cfg.tc);
        if(soil.al!=AL::OK) Draw::badge(88,46,soil.al);
        else { disp.setFont(u8g2_font_5x7_tr); disp.drawStr(90,54,"OK"); }
        Draw::hint("[SEL]Menu [UP/DN]Nav");
    }

    void sens(){
        Draw::statusBar();
        disp.setFont(u8g2_font_6x10_tr); disp.drawStr(1,20,"SENSOR READINGS");
        Draw::divider(22);
        disp.setFont(u8g2_font_5x7_tr);
        char b[24];
        snprintf(b,24,"Moisture : %3d %%",soil.pct); disp.drawStr(2,32,b);
        snprintf(b,24,"Raw ADC  : %4d",  soil.raw);  disp.drawStr(2,41,b);
        snprintf(b,24,"Alert    : %s",Alerts::str(soil.al)); disp.drawStr(2,50,b);
        Draw::hint("[UP/DN]Nav [SEL]Menu");
    }

    void alerts(){
        Draw::statusBar();
        disp.setFont(u8g2_font_6x10_tr); disp.drawStr(1,20,"ALERT STATUS");
        Draw::divider(22);
        disp.setFont(u8g2_font_5x7_tr);
        char b[24];
        if(soil.al==AL::OK){
            disp.drawStr(4,34,"System Normal");
            disp.drawStr(4,45,"No active alerts");
        } else {
            snprintf(b,24,"Level : %s",Alerts::str(soil.al)); disp.drawStr(4,32,b);
            snprintf(b,24,"Moist : %d%%",soil.pct);           disp.drawStr(4,41,b);
            disp.drawStr(4,50,app.aack?"Status: ACK'd":"Status: ACTIVE");
        }
        Draw::hint(soil.al!=AL::OK&&!app.aack?"[SEL]Acknowledge":"[UP/DN]Nav [SEL]Menu");
    }

    void net_(){
        Draw::statusBar();
        disp.setFont(u8g2_font_6x10_tr); disp.drawStr(1,20,"NETWORK");
        Draw::divider(22);
        disp.setFont(u8g2_font_5x7_tr);
        char b[26];
        snprintf(b,26,"Status: %s",FB::stStr());        disp.drawStr(2,31,b);
        if(WiFi.status()==WL_CONNECTED)
            snprintf(b,26,"IP: %s",WiFi.localIP().toString().c_str());
        else strncpy(b,"IP: --",26);
        disp.drawStr(2,40,b);
        snprintf(b,26,"Pushes: %lu",net.pc);            disp.drawStr(2,49,b);
        if(net.lpt) snprintf(b,26,"Last: %lus ago",(millis()-net.lpt)/1000);
        else        strncpy(b,"Last: never",26);
        disp.drawStr(2,54,b); // baseline 54 — just touches BDIV=55, no overlap
        Draw::hint(net.err[0]&&blink ? net.err : "[UP/DN]Nav [SEL]Menu");
    }

    void cfg_(){
        Draw::statusBar();
        disp.setFont(u8g2_font_6x10_tr); disp.drawStr(1,20,"AP CONFIG");
        Draw::divider(22);
        disp.setFont(u8g2_font_5x7_tr);
        if(app.apOn){
            disp.drawStr(2,31,"SSID: " AP_SSID);
            disp.drawStr(2,40,"Pass: " AP_PASS);
            disp.drawStr(2,49,"Open: 192.168.4.1");
            char b[20]; snprintf(b,20,"Clients: %d",net.cli); disp.drawStr(2,54,b);
            Draw::hint("[SEL]Stop AP");
        } else {
            char b[26]; snprintf(b,26,"WiFi:%.19s",cfg.ssid[0]?cfg.ssid:"(not set)");
            disp.drawStr(2,31,b);
            disp.drawStr(2,43,"Press SEL to start");
            disp.drawStr(2,52,"config hotspot");
            Draw::hint("[SEL]Start AP  [UP/DN]Nav");
        }
    }

    void about(){
        Draw::statusBar();
        disp.setFont(u8g2_font_6x10_tr); disp.drawStr(1,20,"ABOUT");
        Draw::divider(22);
        disp.setFont(u8g2_font_5x7_tr);
        disp.drawStr(4,32,DEV_NAME);
        disp.drawStr(4,41,"FW: v" FW_VER);
        disp.drawStr(4,50,"Sensor: Capacitive");
        Draw::hint("[UP/DN]Nav [SEL]Menu");
    }

    // ── Menu overlay ─────────────────────────
    static const char* ML[]={"Dashboard","Sensors","Alerts","Network","AP Config","About"};
    static const uint8_t MN=6,MV=4;
    static uint8_t mscroll=0;

    void menu(){
        disp.setDrawColor(1); disp.drawBox(10,8,108,54);
        disp.setDrawColor(0); disp.drawBox(11,9,106,52);
        disp.setDrawColor(1); disp.drawRFrame(10,8,108,54,2);
        disp.setFont(u8g2_font_6x10_tr); disp.drawStr(14,19,"  MAIN MENU");
        disp.drawHLine(11,21,106);
        uint8_t ve=min((uint8_t)(mscroll+MV),MN);
        for(uint8_t i=mscroll;i<ve;i++){
            uint8_t y=31+(i-mscroll)*11;
            if(i==app.mc){
                disp.drawBox(12,y-8,104,10); disp.setDrawColor(0);
                disp.setFont(u8g2_font_5x7_tr); disp.drawStr(16,y,ML[i]); disp.setDrawColor(1);
            } else { disp.setFont(u8g2_font_5x7_tr); disp.drawStr(16,y,ML[i]); }
        }
        if(MN>MV) disp.drawBox(114,(uint8_t)(22+52*mscroll/MN),2,(uint8_t)(52*MV/MN));
    }

    // Wrap-around menu navigation
    void mup(){
        app.mc=(app.mc==0)?(MN-1):(app.mc-1);
        if(app.mc<mscroll) mscroll=app.mc;
        if(app.mc==MN-1)   mscroll=(MN>MV)?(MN-MV):0;
        Buz::mnav();
    }
    void mdn(){
        app.mc=(app.mc==MN-1)?0:(app.mc+1);
        if(app.mc==0)                mscroll=0;
        else if(app.mc>=mscroll+MV)  mscroll=app.mc-MV+1;
        Buz::mnav();
    }
    void msel(){ app.cur=(View)app.mc; app.mopen=false; Buz::click(); }
}

// ─────────────────────────────────────────────
//  RENDER
// ─────────────────────────────────────────────
void render(){
    disp.clearBuffer();
    switch(app.cur){
        case View::DASH:   Views::dash();   break;
        case View::SENS:   Views::sens();   break;
        case View::ALERTS: Views::alerts(); break;
        case View::NET:    Views::net_();   break;
        case View::CFG:    Views::cfg_();   break;
        case View::ABOUT:  Views::about();  break;
        default: break;
    }
    if(app.mopen) Views::menu();
    disp.sendBuffer();
}

// ─────────────────────────────────────────────
//  INPUT
//  CFG view: short SEL toggles AP (does NOT open menu)
//  All other views: short SEL opens menu
//  Long SEL everywhere: opens menu
//  UP/DOWN: wrap-around view navigation
// ─────────────────────────────────────────────
void handleInput(){
    uint8_t vN=(uint8_t)View::VIEW_COUNT;

    // ── Menu open ────────────────────────────
    if(app.mopen){
        if(Btns::cs(bU)) Views::mup();
        if(Btns::cs(bD)) Views::mdn();
        if(Btns::cs(bS)){ Views::msel(); return; }
        // Close on long SEL
        if(Btns::cl(bS)){ app.mopen=false; Buz::mclose(); }
        return;
    }

    // ── Wrap-around view nav helpers ─────────
    auto prev=[&](){ app.cur=(View)((uint8_t)app.cur==0?vN-1:(uint8_t)app.cur-1); Buz::click(); };
    auto next=[&](){ app.cur=(View)((uint8_t)app.cur==vN-1?0:(uint8_t)app.cur+1); Buz::click(); };

    app.la=millis();

    // ── CFG: short SEL = toggle AP, long SEL = menu ──
    if(app.cur==View::CFG){
        if(Btns::cs(bS)){
            if(app.apOn){ AP::stop(); Buz::mclose(); FB::begin(); }
            else         { AP::begin(); Buz::mopen(); }
            return;
        }
        if(Btns::cl(bS)){ app.mopen=true; app.mc=(uint8_t)app.cur; Buz::mopen(); return; }
        if(Btns::cs(bU)) prev();
        if(Btns::cs(bD)) next();
        return;
    }

    // ── ALERTS: short SEL = ack if active, else menu ──
    if(app.cur==View::ALERTS){
        if(Btns::cs(bS)){
            if(soil.al!=AL::OK&&!app.aack){ app.aack=true; Buz::aack(); }
            else { app.mopen=true; app.mc=(uint8_t)app.cur; Buz::mopen(); }
        }
        if(Btns::cl(bS)){ app.mopen=true; app.mc=(uint8_t)app.cur; Buz::mopen(); }
        if(Btns::cs(bU)) prev();
        if(Btns::cs(bD)) next();
        return;
    }

    // ── All other views: short SEL = menu ────
    if(Btns::cs(bS)){ app.mopen=true; app.mc=(uint8_t)app.cur; Buz::mopen(); }
    if(Btns::cl(bS)){ app.mopen=true; app.mc=(uint8_t)app.cur; Buz::mopen(); }
    if(Btns::cs(bU)) prev();
    if(Btns::cs(bD)) next();
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup(){
    Serial.begin(115200);
    Cfg_::load();
    Buz::begin();
    pinMode(PIN_UP,  INPUT_PULLUP);
    pinMode(PIN_SEL, INPUT_PULLUP);
    pinMode(PIN_DOWN,INPUT_PULLUP);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Wire.begin(I2C_SDA,I2C_SCL);
    disp.begin(); disp.setContrast(200); disp.setDrawColor(1);
    // Boot splash
    disp.clearBuffer();
    disp.setFont(u8g2_font_7x13B_tr); disp.drawStr(8,28,DEV_NAME);
    disp.setFont(u8g2_font_5x7_tr);   disp.drawStr(36,42,"v" FW_VER);
    disp.drawStr(18,54,"Initializing..."); disp.sendBuffer();
    Buz::boot(); delay(400);
    Slp::cfg_wake();
    FB::begin();
    Sens::read(); prevAL=soil.al;
    app.bt=app.la=millis();
}

// ─────────────────────────────────────────────
//  LOOP  (light sleep disabled during AP)
// ─────────────────────────────────────────────
void loop(){
    uint32_t n=millis();
    Btns::poll(PIN_UP,  bU);
    Btns::poll(PIN_SEL, bS);
    Btns::poll(PIN_DOWN,bD);
    handleInput();
    if((n-tSens) >=T_SENSOR) { tSens=n;  Sens::read(); Alerts::update(); }
    if((n-tBlink)>=T_BLINK)  { tBlink=n; blink=!blink; }
    if((n-tDisp) >=T_DISPLAY){ tDisp=n;  render(); }
    if(app.apOn) AP::handle();
    else FB::update();
    // Skip sleep while AP is on — radio must stay awake
    if(!app.apOn){
        bool idle=(n-app.la)>T_IDLE;
        Slp::tick(idle?(uint64_t)T_SENSOR*1000ULL:T_SLEEP);
    }
}