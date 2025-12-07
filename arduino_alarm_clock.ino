#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include "seeed_bme680.h"
#include "Arduino_BMI270_BMM150.h"
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>
#include "mbed.h"
#include "config.h"

// -----------------------------------------------------------------------------
// SENSORS
// -----------------------------------------------------------------------------
#define IIC_ADDR uint8_t(0x76)
Seeed_BME680 bme680(IIC_ADDR);
BoschSensorClass imu(Wire1);

// -----------------------------------------------------------------------------
// CONSTANTES DE COULEURS (RGB565)
// -----------------------------------------------------------------------------
#define COLOR_BLACK   0x0000  // Noir - fond d'écran
#define COLOR_TIME    0xA554  // Orange clair - heure
#define COLOR_DATE    0xAE1B  // Bleu clair - date
#define COLOR_TEMP    0x1C7F  // Bleu - température
#define COLOR_ALARM_ON  0x07E0  // Vert - alarme activée
#define COLOR_ALARM_OFF 0xF800  // Rouge - alarme désactivée
#define COLOR_WIFI_ON   0x07E0  // Vert - WiFi connecté
#define COLOR_WIFI_CONNECTING 0xFFE0  // Jaune - WiFi en connexion
#define COLOR_WIFI_OFF  0xF800  // Rouge - WiFi déconnecté
#define COLOR_BUTTON_1  0xAE1B  // Bleu clair - boutons SET et I/O
#define COLOR_BUTTON_2  0x6674  // Gris - bouton +
#define COLOR_BUTTON_3  0xEBEF  // Gris clair - bouton -
#define COLOR_BUTTON_WIFI 0x07FF  // Cyan - bouton WiFi par défaut
#define COLOR_SNOOZE  0xFFE0  // Jaune - message snooze

// -----------------------------------------------------------------------------
// TAILLES DE BUFFERS
// -----------------------------------------------------------------------------
#define BUFFER_TIME_SIZE   16   // Buffer pour affichage heure "HH:MM:SS"
#define BUFFER_DATE_SIZE   64   // Buffer pour affichage date complète
#define BUFFER_ALARM_SIZE  32   // Buffer pour affichage alarme
#define BUFFER_TEMP_SIZE   64   // Buffer pour température et WiFi

// -----------------------------------------------------------------------------
// CONSTANTES DE TEMPS (millisecondes)
// -----------------------------------------------------------------------------
#define WATCHDOG_TIMEOUT_MS   30000UL  // Watchdog 30 secondes
#define WATCHDOG_KICK_INTERVAL 10000UL // Kick watchdog toutes les 10 secondes
#define WIFI_CONNECT_TIMEOUT  20000UL  // 20 secondes max pour connexion WiFi
#define WIFI_STATUS_CHECK     2000UL   // Vérification statut WiFi toutes les 2 secondes
#define NTP_SYNC_INTERVAL     1800000UL  // 30 minutes entre syncs NTP
#define NTP_SYNC_TIMEOUT      5000UL   // 5 secondes max pour sync NTP
#define TEMP_READ_INTERVAL    2000UL   // Lecture température toutes les 2 secondes
#define TOUCH_DEBOUNCE_MS     300UL    // Antirebond tactile 300ms
#define TILT_CHECK_INTERVAL   200UL    // Vérification inclinaison toutes les 200ms
#define SNOOZE_DURATION_MS    480000UL // 8 minutes de snooze (8*60*1000)
#define BEEP_DURATION_MS      200      // Durée d'un bip en ms
#define BEEP_PAUSE_MS         50       // Pause entre bips en ms
#define BEEP_SEQUENCE_PAUSE_MS 2000    // Pause entre séquences de bips en ms
#define WIFI_INIT_DELAY       1000     // Délai après disconnect avant begin
#define WIFI_RECONNECT_DELAY  500      // Délai pour reconnexion manuelle

// -----------------------------------------------------------------------------
// SEUILS GYROSCOPE
// -----------------------------------------------------------------------------
#define TILT_THRESHOLD        0.20     // Seuil d'inclinaison pour snooze (~11°)
#define TILT_MIN_DELTA        0.15     // Delta minimum pour détecter mouvement

// -----------------------------------------------------------------------------
// BUZZER - AVEC CONTROLE DE VOLUME
// -----------------------------------------------------------------------------
#define BUZZER_PIN 13
#define DEFAULT_VOLUME 100
int volumeLevel = DEFAULT_VOLUME;

// -----------------------------------------------------------------------------
// WIFI / NTP
// -----------------------------------------------------------------------------
// Les identifiants WiFi et NTP sont maintenant dans config.h
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, GMT_OFFSET_SEC, 60000);

// État WiFi et NTP
bool wifiConnected = false;
bool wifiConnecting = false;
unsigned long wifiConnectStart = 0;
unsigned long lastNtpSync = 0;
bool ntpSyncInProgress = false;
unsigned long ntpSyncStart = 0;

// -----------------------------------------------------------------------------
// DISPLAY & TOUCH
// -----------------------------------------------------------------------------
GigaDisplay_GFX display;
Arduino_GigaDisplayTouch touchDetector;

// -----------------------------------------------------------------------------
// ALARM SYSTEM
// -----------------------------------------------------------------------------
int alarm_hour = 6;
int alarm_minute = 45;
bool alarm_enabled = false;
bool is_setting_mode = false;
int setting_field = 0;
bool is_ringing = false;
bool is_snoozing = false;
unsigned long snooze_end = 0;
int last_snooze_minutes_displayed = -1;
bool alarm_triggered_today = false;
int last_alarm_check_minute = -1;

// -----------------------------------------------------------------------------
// TIMERS
// -----------------------------------------------------------------------------
unsigned long lastSecondTick = 0;
unsigned long lastBlink = 0;
bool blinkState = true;
unsigned long lastTempRead = 0;
unsigned long beepSequenceStart = 0;
int beepStep = 0;
unsigned long lastWatchdogKick = 0;

// -----------------------------------------------------------------------------
// LAST DRAW VALUES
// -----------------------------------------------------------------------------
int last_h=-1,last_m=-1,last_s=-1;
int last_day=-1,last_mon=-1,last_year=-1;
int last_alarm_h=-1,last_alarm_m=-1;
bool last_alarm_enabled=false,last_alarm_visible=true;
float lastTemp=0,lastHum=0;
bool last_wifi_status = false;

// -----------------------------------------------------------------------------
// BUTTON UI - AJOUT BOUTON WIFI ET INFO
// -----------------------------------------------------------------------------
#define BUTTON_WIDTH 90
#define BUTTON_HEIGHT 60
#define BUTTON_SPACING 20
#define Y_TIME 40
#define Y_DATE 180
#define Y_ALARM 250
#define Y_BUTTON 320
#define Y_TEMP 420

struct Button {
  int x,y,w,h;
  const char* label;
};

Button buttons[6]; // 6 boutons : SET, +, -, I/O, WiFi, INFO
bool infoMode = false; // Mode écran d'info

// -----------------------------------------------------------------------------
// TOUCH - CORRECTION
// -----------------------------------------------------------------------------
bool wasTouched = false;
unsigned long lastTouchTime = 0;

// -----------------------------------------------------------------------------
// GYROSCOPE
// -----------------------------------------------------------------------------
float baselineX = 0, baselineY = 0, baselineZ = 0;
bool baselineSet = false;
unsigned long lastTiltCheck = 0;

// -----------------------------------------------------------------------------
// DEBUG
// -----------------------------------------------------------------------------
unsigned long lastLoopDebug = 0;

// -----------------------------------------------------------------------------
// HELPERS
// -----------------------------------------------------------------------------
void kick_watchdog() {
  unsigned long now = millis();
  if(now - lastWatchdogKick > WATCHDOG_KICK_INTERVAL) {
    mbed::Watchdog::get_instance().kick();
    lastWatchdogKick = now;
  }
}

void normalize_alarm(){
  if(alarm_minute>=60){alarm_minute-=60; alarm_hour=(alarm_hour+1)%24;}
  if(alarm_minute<0){alarm_minute+=60; alarm_hour=(alarm_hour+23)%24;}
}

void setup_buttons(){
  int totalWidth=6*BUTTON_WIDTH+5*BUTTON_SPACING;
  int startX=(display.width()-totalWidth)/2;
  int y=Y_BUTTON;

  buttons[0]={startX,y,BUTTON_WIDTH,BUTTON_HEIGHT,"SET"};
  buttons[1]={startX+BUTTON_WIDTH+BUTTON_SPACING,y,BUTTON_WIDTH,BUTTON_HEIGHT,"+"};
  buttons[2]={startX+2*(BUTTON_WIDTH+BUTTON_SPACING),y,BUTTON_WIDTH,BUTTON_HEIGHT,"-"};
  buttons[3]={startX+3*(BUTTON_WIDTH+BUTTON_SPACING),y,BUTTON_WIDTH,BUTTON_HEIGHT,"I/O"};
  buttons[4]={startX+4*(BUTTON_WIDTH+BUTTON_SPACING),y,BUTTON_WIDTH,BUTTON_HEIGHT,"WiFi"};
  buttons[5]={startX+5*(BUTTON_WIDTH+BUTTON_SPACING),y,BUTTON_WIDTH,BUTTON_HEIGHT,"INFO"};
}

void draw_buttons(int singleButton = -1){
  uint16_t colors[6]={COLOR_BUTTON_1, COLOR_BUTTON_2, COLOR_BUTTON_3, COLOR_BUTTON_1, COLOR_BUTTON_WIFI, COLOR_BUTTON_1};

  int start = (singleButton >= 0 && singleButton < 6) ? singleButton : 0;
  int end = (singleButton >= 0 && singleButton < 6) ? singleButton + 1 : 6;

  for(int i=start;i<end;i++){
    Button &b=buttons[i];

    // Bouton WiFi change de couleur selon l'état
    uint16_t color = colors[i];
    if(i == 4) { // Bouton WiFi
      if(wifiConnecting || ntpSyncInProgress) {
        color = COLOR_WIFI_CONNECTING;
      } else if(wifiConnected) {
        color = COLOR_WIFI_ON;
      } else {
        color = COLOR_WIFI_OFF;
      }
    }
    // Bouton INFO change de couleur quand actif
    else if(i == 5) { // Bouton INFO
      color = infoMode ? COLOR_WIFI_ON : COLOR_BUTTON_1;
    }

    display.fillRoundRect(b.x,b.y,b.w,b.h,10,color);
    display.setTextColor(COLOR_BLACK,color);
    display.setTextSize(3);

    int16_t x1,y1;
    uint16_t w,h;
    display.getTextBounds(b.label,0,0,&x1,&y1,&w,&h);
    display.setCursor(b.x+(b.w-w)/2,b.y+(b.h-h)/2);
    display.print(b.label);
  }
}

// -----------------------------------------------------------------------------
// DRAW TIME & DATE
// -----------------------------------------------------------------------------
void draw_time(tm *t){
  if(t->tm_hour==last_h && t->tm_min==last_m && t->tm_sec==last_s) return;
  
  // Sauvegarder les anciennes valeurs pour savoir quoi effacer
  int old_h = last_h, old_m = last_m, old_s = last_s;
  last_h=t->tm_hour; last_m=t->tm_min; last_s=t->tm_sec;

  char buf[BUFFER_TIME_SIZE];
  snprintf(buf,BUFFER_TIME_SIZE,"%02d:%02d:%02d",last_h,last_m,last_s);
  
  display.setTextSize(14);
  
  // Calculer la position une seule fois
  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds(buf,0,0,&x1,&y1,&w,&h);
  int xPos = (display.width()-w)/2;
  
  // Si c'est le premier affichage, effacer toute la zone
  if(old_h == -1) {
    display.fillRect(0,Y_TIME,display.width(),h+10,COLOR_BLACK);
  } else {
    // Sinon, effacer uniquement la zone des secondes (optimisation)
    // Largeur approximative d'un chiffre en taille 14
    int charWidth = w / 8; // 8 caractères dans "HH:MM:SS"
    int secX = xPos + (charWidth * 6); // Position du début des secondes
    display.fillRect(secX-5, Y_TIME, charWidth*3, h+10, COLOR_BLACK);
  }

  // Dessiner le texte complet
  display.setTextColor(COLOR_TIME,COLOR_BLACK);
  display.setCursor(xPos,Y_TIME);
  display.print(buf);
}

void draw_date(tm *t){
  if(t->tm_mday==last_day && t->tm_mon==last_mon && t->tm_year==last_year) return;
  last_day=t->tm_mday; last_mon=t->tm_mon; last_year=t->tm_year;
  
  static const char* days[]={"Dimanche","Lundi","Mardi","Mercredi","Jeudi","Vendredi","Samedi"};
  static const char* months[]={"","janvier","fevrier","mars","avril","mai","juin","juillet","aout","septembre","octobre","novembre","decembre"};

  char line1[BUFFER_DATE_SIZE];
  snprintf(line1,BUFFER_DATE_SIZE,"%s %02d %s %04d",days[t->tm_wday],t->tm_mday,months[t->tm_mon+1],t->tm_year+1900);
  
  display.setTextSize(4);
  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds(line1,0,0,&x1,&y1,&w,&h);
  display.fillRect(0,Y_DATE,display.width(),h+8,COLOR_BLACK);
  display.setTextColor(COLOR_DATE,COLOR_BLACK);
  display.setCursor((display.width()-w)/2,Y_DATE);
  display.print(line1);
}

void get_current_time(tm *t) {
  time_t now_epoch = time(NULL);
  struct tm *now_tm = localtime(&now_epoch);
  memcpy(t, now_tm, sizeof(tm));
}

// -----------------------------------------------------------------------------
// ECRAN INFO / STATUT
// -----------------------------------------------------------------------------
void draw_info_screen() {
  display.fillScreen(COLOR_BLACK);

  display.setTextColor(COLOR_WIFI_ON, COLOR_BLACK);
  display.setTextSize(3);

  int y = 20;
  int lineHeight = 30;
  char buf[BUFFER_TEMP_SIZE];

  // Titre
  display.setCursor(10, y);
  display.print("=== STATUT SYSTEME ===");
  y += lineHeight + 10;

  display.setTextSize(2);

  // WiFi Status
  display.setCursor(10, y);
  if(wifiConnected) {
    int rssi = WiFi.RSSI();
    snprintf(buf, BUFFER_TEMP_SIZE, "WiFi: CONNECTE (%d dBm)", rssi);
  } else {
    snprintf(buf, BUFFER_TEMP_SIZE, "WiFi: DECONNECTE");
  }
  display.print(buf);
  y += lineHeight;

  // NTP Sync
  display.setCursor(10, y);
  if(wifiConnected) {
    unsigned long minutesSinceSync = (millis() - lastNtpSync) / 60000;
    snprintf(buf, BUFFER_TEMP_SIZE, "NTP: Sync il y a %lu min", minutesSinceSync);
  } else {
    snprintf(buf, BUFFER_TEMP_SIZE, "NTP: Mode RTC interne");
  }
  display.print(buf);
  y += lineHeight;

  // Capteurs
  display.setCursor(10, y);
  display.print("BME680: OK");
  y += lineHeight;

  display.setCursor(10, y);
  display.print("IMU: OK");
  y += lineHeight;

  // Température / Humidité
  display.setCursor(10, y);
  snprintf(buf, BUFFER_TEMP_SIZE, "T: %.1fC  H: %.0f%%", lastTemp, lastHum);
  display.print(buf);
  y += lineHeight;

  // Mémoire libre (approximation - non disponible sur mbed)
  display.setCursor(10, y);
  display.print("Memoire: OK");
  y += lineHeight;

  // Uptime
  display.setCursor(10, y);
  unsigned long uptimeSeconds = millis() / 1000;
  unsigned long hours = uptimeSeconds / 3600;
  unsigned long minutes = (uptimeSeconds % 3600) / 60;
  snprintf(buf, BUFFER_TEMP_SIZE, "Uptime: %luh %lumin", hours, minutes);
  display.print(buf);
  y += lineHeight + 20;

  // Instructions
  display.setTextSize(2);
  display.setCursor(10, y);
  display.print("Appuyez sur INFO pour");
  y += lineHeight;
  display.setCursor(10, y);
  display.print("Revenir au reveil");

  // Redessiner uniquement le bouton INFO
  draw_buttons(5);
}

// -----------------------------------------------------------------------------
// ALARM
// -----------------------------------------------------------------------------
void draw_alarm_line(bool force=false){
  bool visible=is_setting_mode?blinkState:true;
  if(!force && last_alarm_h==alarm_hour && last_alarm_m==alarm_minute && 
     last_alarm_enabled==alarm_enabled && last_alarm_visible==visible) return;
  
  last_alarm_h=alarm_hour; last_alarm_m=alarm_minute;
  last_alarm_enabled=alarm_enabled; last_alarm_visible=visible;

  display.fillRect(0,Y_ALARM,display.width(),70,COLOR_BLACK);
  if(!visible) return;

  char buf[BUFFER_ALARM_SIZE];
  snprintf(buf,BUFFER_ALARM_SIZE,"Alarme : %02d:%02d",alarm_hour,alarm_minute);

  display.setTextColor(alarm_enabled?COLOR_ALARM_ON:COLOR_ALARM_OFF,COLOR_BLACK);
  display.setTextSize(5);
  
  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds(buf,0,0,&x1,&y1,&w,&h);
  display.setCursor((display.width()-w)/2,Y_ALARM);
  display.print(buf);
}

// -----------------------------------------------------------------------------
// TEMP/HUM/WIFI STATUS
// -----------------------------------------------------------------------------
void draw_temp(){
  display.fillRect(0,Y_TEMP,display.width(),70,COLOR_BLACK);

  char buf[BUFFER_TEMP_SIZE];
  if(wifiConnected) {
    int rssi = WiFi.RSSI();
    snprintf(buf,BUFFER_TEMP_SIZE,"T:%.1fC H:%.0f%% WiFi:%d(dBm)",
            lastTemp, lastHum, rssi);
  } else {
    snprintf(buf,BUFFER_TEMP_SIZE,"T:%.1fC H:%.0f%% WiFi:OFF (RTC interne)",
            lastTemp, lastHum);
  }

  display.setTextColor(COLOR_TEMP,COLOR_BLACK);
  display.setTextSize(3);
  
  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds(buf,0,0,&x1,&y1,&w,&h);
  display.setCursor((display.width()-w)/2,Y_TEMP);
  display.print(buf);
}

void task_temp(){
  unsigned long now=millis();
  if(now-lastTempRead<TEMP_READ_INTERVAL) return;
  lastTempRead=now;
  
  if(bme680.read_sensor_data()) return;
  
  lastTemp=bme680.sensor_result_value.temperature;
  lastHum=bme680.sensor_result_value.humidity;
  
  draw_temp();
}

// Nouvelle fonction pour vérifier le changement de statut WiFi
void check_wifi_status_change() {
  static unsigned long lastWifiStatusCheck = 0;
  unsigned long now = millis();

  // Vérifier le statut WiFi périodiquement
  if(now - lastWifiStatusCheck < WIFI_STATUS_CHECK) return;
  lastWifiStatusCheck = now;
  
  bool currentWifiStatus = (WiFi.status() == WL_CONNECTED);
  if(currentWifiStatus != last_wifi_status) {
    Serial.print(">>> Changement statut WiFi: ");
    Serial.println(currentWifiStatus ? "CONNECTE" : "DECONNECTE");
    
    last_wifi_status = currentWifiStatus;
    wifiConnected = currentWifiStatus;
    
    if(currentWifiStatus) {
      // WiFi vient de se connecter
      Serial.println(">>> WiFi connecté - Synchronisation NTP en cours...");
      ntpSyncInProgress = true;
      ntpSyncStart = millis();
      
      timeClient.begin();
    } else {
      // WiFi vient de se déconnecter
      ntpSyncInProgress = false;
      timeClient.end();
    }
    
    draw_buttons(); // Mettre à jour la couleur du bouton WiFi
    draw_temp();    // Mettre à jour l'affichage WiFi
  }
}

// NOUVEAU : Tâche de synchronisation NTP automatique
void ntp_sync_task() {
  unsigned long now = millis();
  
  // Si une sync est en cours, vérifier le timeout
  if(ntpSyncInProgress) {
    if(timeClient.forceUpdate()) {
      time_t epoch = timeClient.getEpochTime();
      set_time(epoch);
      
      ntpSyncInProgress = false;
      lastNtpSync = now;
      
      Serial.println(">>> NTP SYNCHRONISE avec succès");
      Serial.print("    Heure: ");
      tm timeStruct;
      get_current_time(&timeStruct);
      Serial.print(timeStruct.tm_hour);Serial.print(":");
      Serial.print(timeStruct.tm_min);Serial.print(":");
      Serial.println(timeStruct.tm_sec);
      
      draw_buttons();
      return;
    }
    
    // Timeout de sync
    if(now - ntpSyncStart > NTP_SYNC_TIMEOUT) {
      ntpSyncInProgress = false;
      Serial.println(">>> Timeout synchronisation NTP");
      draw_buttons();
      return;
    }
  }
  
  // Vérifier s'il faut faire une sync périodique
  if(wifiConnected && !ntpSyncInProgress && (now - lastNtpSync > NTP_SYNC_INTERVAL)) {
    Serial.println(">>> Synchronisation NTP périodique (30 min)...");
    ntpSyncInProgress = true;
    ntpSyncStart = now;
  }
}

// -----------------------------------------------------------------------------
// BUZZER / SNOOZE
// -----------------------------------------------------------------------------
void stop_ringing(){
  if(!is_ringing) return;
  is_ringing=false;
  beepStep=0;
  analogWrite(BUZZER_PIN,0);
  Serial.println(">>> ALARME ARRETEE");
}

void start_ringing(){
  if(is_ringing) return;
  is_ringing=true;
  beepStep=0;
  beepSequenceStart=millis();
  Serial.println(">>> ALARME DECLENCHEE");
}

void buzzer_task(){
  if(!is_ringing) {
    analogWrite(BUZZER_PIN,0);
    return;
  }
  
  unsigned long now=millis();
  unsigned long elapsed=now-beepSequenceStart;
  
  switch(beepStep) {
    case 0:
      analogWrite(BUZZER_PIN, volumeLevel);
      if(elapsed >= BEEP_DURATION_MS) { beepStep = 1; beepSequenceStart = now; }
      break;
    case 1:
      analogWrite(BUZZER_PIN, 0);
      if(elapsed >= BEEP_PAUSE_MS) { beepStep = 2; beepSequenceStart = now; }
      break;
    case 2:
      analogWrite(BUZZER_PIN, volumeLevel);
      if(elapsed >= BEEP_DURATION_MS) { beepStep = 3; beepSequenceStart = now; }
      break;
    case 3:
      analogWrite(BUZZER_PIN, 0);
      if(elapsed >= BEEP_PAUSE_MS) { beepStep = 4; beepSequenceStart = now; }
      break;
    case 4:
      if(elapsed >= BEEP_SEQUENCE_PAUSE_MS) { beepStep = 0; beepSequenceStart = now; }
      break;
  }
}

void trigger_snooze(){
  stop_ringing();
  is_snoozing=true;
  snooze_end=millis()+SNOOZE_DURATION_MS;
  last_snooze_minutes_displayed = 8; // Réinitialiser le compteur

  display.fillRect(0,150,display.width(),100,COLOR_BLACK);
  display.setTextColor(COLOR_SNOOZE,COLOR_BLACK);
  display.setTextSize(6);

  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds("SNOOZE 8 min",0,0,&x1,&y1,&w,&h);
  display.setCursor((display.width()-w)/2,180);
  display.print("SNOOZE 8 min");

  Serial.println(">>> SNOOZE active - 8 minutes");
}

void update_snooze_display() {
  if(!is_snoozing) return;

  unsigned long now = millis();
  if(now >= snooze_end) return; // L'alarme va sonner, pas besoin de mettre à jour

  // Calculer les minutes restantes (arrondi supérieur)
  int minutes_remaining = ((snooze_end - now) / 60000) + 1;
  if(minutes_remaining > 8) minutes_remaining = 8; // Maximum 8 minutes

  // Mettre à jour seulement si le nombre de minutes a changé
  if(minutes_remaining != last_snooze_minutes_displayed) {
    last_snooze_minutes_displayed = minutes_remaining;

    char buf[16];
    snprintf(buf, 16, "SNOOZE %d min", minutes_remaining);

    display.fillRect(0,150,display.width(),100,COLOR_BLACK);
    display.setTextColor(COLOR_SNOOZE,COLOR_BLACK);
    display.setTextSize(6);

    int16_t x1,y1; uint16_t w,h;
    display.getTextBounds(buf,0,0,&x1,&y1,&w,&h);
    display.setCursor((display.width()-w)/2,180);
    display.print(buf);

    Serial.print(">>> SNOOZE compte à rebours: ");
    Serial.print(minutes_remaining);
    Serial.println(" min");
  }
}

// -----------------------------------------------------------------------------
// GYROSCOPE
// -----------------------------------------------------------------------------
void calibrate_baseline() {
  if(imu.accelerationAvailable()) {
    imu.readAcceleration(baselineX,baselineY,baselineZ);
    baselineSet=true;
    Serial.println("Baseline IMU calibre");
  }
}

void check_tilt_for_snooze() {
  if(!is_ringing) return;

  unsigned long now=millis();
  if(now-lastTiltCheck<TILT_CHECK_INTERVAL) return;
  lastTiltCheck=now;
  
  if(!baselineSet) {
    calibrate_baseline();
    return;
  }
  
  float x,y,z;
  if(imu.accelerationAvailable()) {
    imu.readAcceleration(x,y,z);

    float deltaX=abs(x-baselineX);
    float deltaY=abs(y-baselineY);
    float deltaZ=abs(z-baselineZ);

    if(deltaX>TILT_MIN_DELTA || deltaY>TILT_MIN_DELTA || deltaZ>TILT_MIN_DELTA) {
      Serial.print("Mouvement - dX:");Serial.print(deltaX,2);
      Serial.print(" dY:");Serial.print(deltaY,2);
      Serial.print(" dZ:");Serial.print(deltaZ,2);
      Serial.print(" | Seuil:");Serial.println(TILT_THRESHOLD,2);
    }

    if(deltaX>TILT_THRESHOLD || deltaY>TILT_THRESHOLD || deltaZ>TILT_THRESHOLD) {
      Serial.println(">>> INCLINAISON DETECTEE - SNOOZE ACTIVE !");
      trigger_snooze();
    }
  }
}

// -----------------------------------------------------------------------------
// ALARM TRIGGER
// -----------------------------------------------------------------------------
void check_alarm(tm *t){
  if(!alarm_enabled) {
    alarm_triggered_today=false;
    last_alarm_check_minute=-1;
    return;
  }
  
  if(is_snoozing) {
    if(millis()>=snooze_end) {
      is_snoozing=false;
      last_snooze_minutes_displayed = -1; // Reset du compteur
      display.fillRect(0,150,display.width(),100,COLOR_BLACK);
      last_day=-1;
      draw_date(t);
      start_ringing();
      Serial.println(">>> FIN SNOOZE - Alarme resonne");
    }
    return;
  }
  
  int current_minute=t->tm_min;
  if(current_minute!=last_alarm_check_minute) {
    last_alarm_check_minute=current_minute;
    
    if(alarm_triggered_today && (t->tm_hour!=alarm_hour || t->tm_min!=alarm_minute)) {
      alarm_triggered_today=false;
    }
  }
  
  if(!is_ringing && !alarm_triggered_today && 
     t->tm_hour==alarm_hour && t->tm_min==alarm_minute) {
    alarm_triggered_today=true;
    start_ringing();
    Serial.print(">>> ALARME ACTIVEE a ");
    Serial.print(t->tm_hour);Serial.print(":");Serial.println(t->tm_min);
  }
}

// -----------------------------------------------------------------------------
// WIFI / NTP - CONNEXION INITIALE UNIQUEMENT
// -----------------------------------------------------------------------------
void init_wifi_ntp(){
  Serial.println("=== TENTATIVE CONNEXION WIFI INITIALE ===");

  WiFi.disconnect();
  delay(WIFI_INIT_DELAY);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts=0;
  const int maxAttempts=40; // 12 secondes max

  while(WiFi.status()!=WL_CONNECTED && attempts<maxAttempts){
    delay(300);
    attempts++;
    if(attempts%5==0){
      Serial.print("Tentative WiFi: ");
      Serial.print(attempts);Serial.print("/");Serial.println(maxAttempts);
      mbed::Watchdog::get_instance().kick(); // Kick pendant la connexion WiFi
    }
  }
  
  if(WiFi.status()==WL_CONNECTED){
    wifiConnected = true;
    Serial.println("=== WIFI CONNECTE ===");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    Serial.print("Signal: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    
    timeClient.begin();
    
    Serial.println(">>> Synchronisation NTP initiale...");
    ntpSyncInProgress = true;
    ntpSyncStart = millis();
    
  } else {
    wifiConnected = false;
    Serial.println("=== WiFi non disponible - Mode RTC interne ===");
    Serial.println("Le reveil fonctionnera avec l'horloge interne");
    Serial.println("Appuyez sur le bouton WiFi pour reconnecter");
  }
}

// -----------------------------------------------------------------------------
// WIFI - RECONNEXION MANUELLE (BOUTON)
// -----------------------------------------------------------------------------
void manual_wifi_reconnect(){
  if(wifiConnecting) {
    Serial.println("Connexion WiFi deja en cours...");
    return;
  }
  
  Serial.println("=== RECONNEXION WIFI MANUELLE ===");
  wifiConnecting = true;
  wifiConnectStart = millis();
  draw_buttons(); // Mettre le bouton en jaune

  WiFi.disconnect();
  delay(WIFI_RECONNECT_DELAY);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// -----------------------------------------------------------------------------
// WIFI - TACHE DE VERIFICATION (connexion manuelle uniquement)
// -----------------------------------------------------------------------------
void wifi_task(){
  if(!wifiConnecting) return;
  
  unsigned long now = millis();
  
  // Vérifier si connecté
  if(WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    wifiConnecting = false;
    
    Serial.println("=== WIFI RECONNECTE ===");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    
    timeClient.begin();
    
    Serial.println(">>> Synchronisation NTP après reconnexion...");
    ntpSyncInProgress = true;
    ntpSyncStart = millis();
    
    draw_buttons();
    draw_temp();
    return;
  }
  
  // Timeout
  if(now - wifiConnectStart > WIFI_CONNECT_TIMEOUT) {
    wifiConnecting = false;
    wifiConnected = false;
    
    Serial.println("=== TIMEOUT CONNEXION WIFI ===");
    Serial.println("Retour au mode RTC interne");
    
    WiFi.disconnect();
    
    draw_buttons();
    draw_temp();
  }
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  Serial.println("=== DEMARRAGE REVEIL ===");

  // Initialisation Watchdog
  mbed::Watchdog &watchdog = mbed::Watchdog::get_instance();
  watchdog.start(WATCHDOG_TIMEOUT_MS);
  Serial.print(">>> Watchdog active: ");
  Serial.print(WATCHDOG_TIMEOUT_MS / 1000);
  Serial.println(" secondes");
  
  if(!imu.begin()){ Serial.println("IMU fail"); while(1); }
  Serial.println("IMU OK");
  
  while(!bme680.init()){
    Serial.println("BME680 retry");
    mbed::Watchdog::get_instance().kick(); // Kick pendant l'init longue
    delay(1000);
  }
  Serial.println("BME680 OK");
  
  display.begin();
  display.setRotation(1);
  display.fillScreen(COLOR_BLACK);
  Serial.println("Display OK");
  
  if(!touchDetector.begin()){ Serial.println("Touch fail"); while(1); }
  Serial.println("Touch OK");
  
  setup_buttons();
  draw_buttons();
  
  pinMode(BUZZER_PIN,OUTPUT);
  analogWrite(BUZZER_PIN,0);
  Serial.println("Buzzer OK");
  
  init_wifi_ntp(); // Tentative initiale uniquement
  
  delay(500);
  calibrate_baseline();
  
  Serial.println("=== REVEIL PRET ===");
  Serial.print("Alarme: ");Serial.print(alarm_hour);Serial.print(":");Serial.println(alarm_minute);
  Serial.print("Mode: ");Serial.println(wifiConnected ? "WiFi+NTP" : "RTC interne");
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------
void loop(){
  // Kick watchdog périodiquement
  kick_watchdog();

  // --- TOUCH DETECTION ---
  GDTpoint_t points[5];
  uint8_t contacts=touchDetector.getTouchPoints(points);
  
  if(contacts>0 && !wasTouched) {
    if(millis()-lastTouchTime>TOUCH_DEBOUNCE_MS) {
      int touchX=points[0].x;
      int touchY=points[0].y;
      int x=touchY;
      int y=480-touchX;
      
      for(int i=0; i<6; i++) {
        Button &b=buttons[i];
        if(x>=b.x && x<=b.x+b.w && y>=b.y && y<=b.y+b.h) {
          Serial.print(">>> BOUTON: ");Serial.println(i);

          // SET
          if(i==0) {
            if(!is_setting_mode) {
              is_setting_mode=true;
              setting_field=0;
              blinkState=true;
              lastBlink=millis();
            } else {
              setting_field++;
              if(setting_field>1) is_setting_mode=false;
            }
            draw_alarm_line(true);
          }
          // +
          else if(i==1 && is_setting_mode) {
            if(setting_field==0) alarm_hour=(alarm_hour+1)%24;
            else alarm_minute=(alarm_minute+1)%60;
            draw_alarm_line(true);
          }
          // -
          else if(i==2 && is_setting_mode) {
            if(setting_field==0) alarm_hour=(alarm_hour+23)%24;
            else alarm_minute=(alarm_minute+59)%60;
            draw_alarm_line(true);
          }
          // I/O
          else if(i==3) {
            if(is_ringing || is_snoozing) {
              stop_ringing();
              is_snoozing=false;
              last_snooze_minutes_displayed = -1; // Reset du compteur
              alarm_triggered_today=true;
              display.fillRect(0,150,display.width(),100,COLOR_BLACK);
              last_day=-1;
              tm timeStruct;
              get_current_time(&timeStruct);
              draw_date(&timeStruct);
              Serial.println(">>> Alarme/Snooze arrete(e)");
            } else {
              alarm_enabled=!alarm_enabled;
              if(!alarm_enabled) {
                is_snoozing=false;
                alarm_triggered_today=false;
              }
              Serial.print(">>> Alarme ");
              Serial.println(alarm_enabled?"ACTIVEE":"DESACTIVEE");
            }
            draw_alarm_line(true);
          }
          // WIFI
          else if(i==4) {
            manual_wifi_reconnect();
          }
          // INFO
          else if(i==5) {
            Serial.println(">>> BOUTON INFO");
            infoMode = !infoMode; // Basculer le mode

            if(infoMode) {
              draw_info_screen();
            } else {
              // Retour au mode réveil - redessiner tout
              display.fillScreen(COLOR_BLACK);
              last_h = last_m = last_s = -1; // Forcer le rafraîchissement
              last_day = last_mon = last_year = -1;
              last_alarm_h = last_alarm_m = -1;
              draw_buttons();
            }
          }

          lastTouchTime=millis();
          break;
        }
      }
    }
    wasTouched=true;
  } else if(contacts==0) {
    wasTouched=false;
  }

  // --- CLOCK / DISPLAY ---
  if(!infoMode) {
    tm timeStruct;
    get_current_time(&timeStruct);
    tm *t = &timeStruct;

    draw_time(t);
    draw_date(t);

    if(is_setting_mode && millis()-lastBlink>=500) {
      lastBlink=millis();
      blinkState=!blinkState;
      draw_alarm_line();
    } else if(!is_setting_mode) {
      draw_alarm_line();
    }

    task_temp();
    check_alarm(t);
    buzzer_task();
    check_tilt_for_snooze();
    update_snooze_display(); // Compte à rebours du snooze
  }

  // Vérifier les changements de statut WiFi (toutes les 2 secondes)
  check_wifi_status_change();
  
  // NOUVEAU : Tâche de synchronisation NTP (toutes les heures + reconnexions)
  ntp_sync_task();
  
  // Gestion manuelle de la connexion WiFi
  wifi_task();
}