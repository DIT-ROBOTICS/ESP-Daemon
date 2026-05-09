#include "wifi_config.h"

#include "config.h"
#include "app_settings.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <cstring>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <NetWizard.h>
#include <esp_wifi.h>

int wifi_channel = 0;

namespace {

AsyncWebServer netWizardServer(80);
NetWizard netWizard(&netWizardServer);
bool netWizardConfigured              = false;
bool mdnsStarted                      = false;
bool netWizardHttpReleased            = false;
bool restartAfterProvisioningPending  = false;
unsigned long restartAfterProvisioningStartedMs = 0;
unsigned long restartAfterProvisioningDelayMs   = 0;
String mdnsName;

String getProvisioningSsid() {
  const uint64_t efuse = ESP.getEfuseMac();
  const uint32_t macTail = static_cast<uint32_t>(efuse & 0xFFFFFFULL);
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%06X", static_cast<unsigned>(macTail));
#if APP_MODE == APP_MODE_ESTOP
  return String("ESP-EStop_") + suffix;
#else
  return String("ESP-Daemon_") + suffix;
#endif
}

String getDeviceNameLower() {
  // device_name comes from Settings and is sanitized in app_settings.cpp.
  // Re-apply lowercase here to ensure WiFi/MDNS match exactly.
  AppSettingsReadGuard settingsGuard;
  String host = settingsGuard.settings().device_name;
  host.trim();
  host.toLowerCase();
  if (host.length() == 0) {
    // Fallback: avoid empty hostname/MDNS.
    host = getProvisioningSsid();
    host.toLowerCase();
  }
  return host;
}

void applyMdnsName() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  const String requestedName = getDeviceNameLower();
  if (mdnsStarted && requestedName == mdnsName) {
    return;
  }

  if (mdnsStarted) {
    MDNS.end();
    mdnsStarted = false;
  }

  if (MDNS.begin(requestedName.c_str())) {
    mdnsStarted = true;
    mdnsName = requestedName;
    DAEMON_LOGF("mDNS started: %s.local\n", requestedName.c_str());
  } else {
    DAEMON_LOGLN("mDNS init failed");
  }
}

void ensurePortalIfDisconnected() {
  static    unsigned long lastPortalStartMs         = 0;
  static    unsigned long disconnectedSinceMs       = 0;
  static    unsigned long lastReconnectOnlyLogMs    = 0;
  constexpr unsigned long kPortalRestartCooldownMs  = 30000;
  constexpr unsigned long kPortalStartGraceMs       = 45000;
  constexpr unsigned long kPortalForceWithCredsMs   = 300000;

  if (WiFi.status() == WL_CONNECTED) {
    disconnectedSinceMs = 0;
    return;
  }

  // Once disconnected, allow NetWizard HTTP server to be resumed by portal.
  netWizardHttpReleased = false;

  if (disconnectedSinceMs == 0) {
    disconnectedSinceMs = millis();
  }

  // If AP mode is already active, avoid repeatedly restarting the portal.
  if ((WiFi.getMode() & WIFI_AP) != 0) {
    return;
  }

  // Give STA reconnect some time before re-opening the captive portal.
  if ((millis() - disconnectedSinceMs) < kPortalStartGraceMs) {
    return;
  }

  const unsigned long now = millis();
  if ((now - lastPortalStartMs) < kPortalRestartCooldownMs) {
    return;
  }

  // If credentials exist, prioritize silent auto-reconnect and avoid forcing
  // users back into captive portal after successful provisioning.
  const char* savedSsid = netWizard.getSSID();
  if (savedSsid != nullptr && std::strlen(savedSsid) > 0) {
    const unsigned long disconnectedForMs = now - disconnectedSinceMs;
    if (disconnectedForMs < kPortalForceWithCredsMs) {
      if ((now - lastReconnectOnlyLogMs) >= 30000) {
        lastReconnectOnlyLogMs = now;
        DAEMON_LOGF("WiFi disconnected, retrying saved SSID (%s), portal suppressed\n", savedSsid);
      }
      return;
    }

    if ((now - lastReconnectOnlyLogMs) >= 30000) {
      lastReconnectOnlyLogMs = now;
      DAEMON_LOGF("WiFi disconnected for %lus, forcing portal despite saved SSID (%s)\n",
                  disconnectedForMs / 1000UL, savedSsid);
    }
  }

  if (netWizard.getPortalState() == NetWizardPortalState::IDLE) {
    lastPortalStartMs = now;
    DAEMON_LOGLN("NetWizard portal restart requested");
    netWizard.startPortal();
  }
}

void reconnectSavedWifiIfDisconnected() {
  static    unsigned long lastReconnectAttemptMs      = 0;
  constexpr unsigned long kReconnectAttemptIntervalMs = 10000;

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  // Avoid interfering when NetWizard is currently handling a portal connect flow.
  const NetWizardPortalState portalState = netWizard.getPortalState();
  if (portalState == NetWizardPortalState::CONNECTING_WIFI ||
      portalState == NetWizardPortalState::WAITING_FOR_CONNECTION) {
    return;
  }

  const char* ssid = netWizard.getSSID();
  if (ssid == nullptr || std::strlen(ssid) == 0) {
    return;
  }

  const unsigned long now = millis();
  if ((now - lastReconnectAttemptMs) < kReconnectAttemptIntervalMs) {
    return;
  }
  lastReconnectAttemptMs = now;

  const wifi_mode_t mode = WiFi.getMode();
  if ((mode & WIFI_AP) != 0) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_STA);
  }

  String deviceName = getDeviceNameLower();
  WiFi.setHostname(deviceName.c_str());

  if (netWizard.connect()) {
    DAEMON_LOGF("WiFi reconnect attempt -> %s\n", ssid);
  }
}

void applyWifiStaLinkProfile(bool force = false) {
  static    unsigned long lastAttemptMs = 0;
  static    bool          lastAppliedOk = false;
  constexpr unsigned long kRetryMs      = 5000;

  const unsigned long now = millis();
  if (!force && (now - lastAttemptMs) < kRetryMs) {
    return;
  }
  lastAttemptMs = now;

  if ((WiFi.getMode() & WIFI_STA) == 0) {
    return;
  }

  // Keep latency predictable under weak links.
  WiFi.setSleep(false);

  const esp_err_t psErr = esp_wifi_set_ps(WIFI_PS_NONE);
  const esp_err_t bwErr = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);

  wifi_bandwidth_t currentBw = WIFI_BW_HT20;
  const esp_err_t readBwErr = esp_wifi_get_bandwidth(WIFI_IF_STA, &currentBw);
  const bool ok = (psErr == ESP_OK) && (bwErr == ESP_OK) && (readBwErr == ESP_OK) && (currentBw == WIFI_BW_HT20);

  if (ok && !lastAppliedOk) {
    DAEMON_LOGLN("WiFi STA profile applied: PS=NONE, BW=HT20");
  } else if (!ok) {
    DAEMON_LOGF("WiFi STA profile pending (ps=%d bw=%d getbw=%d curbw=%d)\n",
                static_cast<int>(psErr),
                static_cast<int>(bwErr),
                static_cast<int>(readBwErr),
                static_cast<int>(currentBw));
  }
  lastAppliedOk = ok;
}

void configureNetWizard() {
  if (netWizardConfigured) {
    return;
  }

  netWizard.setStrategy(NetWizardStrategy::NON_BLOCKING);
  netWizard.setConnectTimeout(5000);
  // NetWizard treats timeout=0 as immediate timeout. Use a very large timeout
  // to keep portal available effectively "forever" for field provisioning.
  netWizard.setPortalTimeout(0xFFFFFFFFUL);
  netWizard.onPortalState([](NetWizardPortalState state) {
    switch (state) {
      case NetWizardPortalState::IDLE:
        DAEMON_LOGLN("NetWizard portal state: IDLE");
        break;
      case NetWizardPortalState::CONNECTING_WIFI:
        DAEMON_LOGLN("NetWizard portal state: CONNECTING_WIFI");
        break;
      case NetWizardPortalState::WAITING_FOR_CONNECTION:
        DAEMON_LOGLN("NetWizard portal state: WAITING_FOR_CONNECTION");
        break;
      case NetWizardPortalState::SUCCESS:
        DAEMON_LOGLN("NetWizard portal state: SUCCESS");
        if (!restartAfterProvisioningPending) {
          constexpr unsigned long kRestartDelayMs = 1500;
          restartAfterProvisioningPending = true;
          restartAfterProvisioningStartedMs = millis();
          restartAfterProvisioningDelayMs   = kRestartDelayMs;
          DAEMON_LOGLN("Provisioning success -> reboot scheduled");
        }
        break;
      case NetWizardPortalState::FAILED:
        DAEMON_LOGLN("NetWizard portal state: FAILED");
        break;
      case NetWizardPortalState::TIMEOUT:
        DAEMON_LOGLN("NetWizard portal state: TIMEOUT");
        break;
      default:
        break;
    }
  });
  netWizardConfigured = true;
}

void handleProvisioningRestart() {
  if (!restartAfterProvisioningPending) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  const unsigned long now = millis();
  if ((now - restartAfterProvisioningStartedMs) < restartAfterProvisioningDelayMs) {
    return;
  }

  DAEMON_LOGLN("Rebooting to apply network state cleanly...");
  delay(50);
  ESP.restart();
}

void releaseNetWizardHttpIfConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (netWizardHttpReleased) {
    return;
  }

  // Ensure portal/AP mode is closed after successful provisioning.
  if ((WiFi.getMode() & WIFI_AP) != 0 || netWizard.getPortalState() != NetWizardPortalState::IDLE) {
    netWizard.stopPortal();
  }

  // NetWizard currently removes handlers but does not always close the
  // AsyncWebServer listener. Release port 80 explicitly so the main WebServer
  // can serve index/settings reliably.
  netWizardServer.end();
  netWizardHttpReleased = true;
  DAEMON_LOGLN("NetWizard HTTP listener released (port 80)");
}

void updateWifiChannelCache() {
  if (WiFi.status() == WL_CONNECTED) {
    wifi_channel = WiFi.channel();
    return;
  }
  AppSettingsReadGuard settingsGuard;
  wifi_channel = settingsGuard.settings().espnow_channel;
}

void persistEspNowChannelFromConnectedWifi() {
#if APP_MODE == APP_MODE_ESTOP
  static int lastSavedChannel = -1;

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  const int connectedChannel = WiFi.channel();
  if (connectedChannel < 1 || connectedChannel > 13) {
    return;
  }

  {
    AppSettingsReadGuard settingsGuard;
    if (settingsGuard.settings().espnow_channel == connectedChannel) {
      lastSavedChannel = connectedChannel;
      return;
    }
  }

  if (lastSavedChannel == connectedChannel) {
    return;
  }

  JsonDocument patch;
  patch["espNowChannel"] = connectedChannel;

  String saveError;
  if (updateAppSettingsFromJson(patch.as<JsonObjectConst>(), saveError)) {
    lastSavedChannel = connectedChannel;
    DAEMON_LOGF("E-stop stored WiFi channel for ESP-NOW: %d\n", connectedChannel);
  } else {
    DAEMON_LOGF("E-stop failed to store WiFi channel (%d): %s\n",
                connectedChannel, saveError.c_str());
  }
#endif
}

} // namespace

void initWiFi() {
  configureNetWizard();

  String deviceName = getDeviceNameLower();
  String provisioningSsid = getProvisioningSsid();
  netWizard.setHostname(deviceName.c_str());

  WiFi.setSleep(false);
  applyWifiStaLinkProfile(true);

  DAEMON_LOGF("Starting NetWizard auto-connect (AP SSID: %s, open AP)\n", provisioningSsid.c_str());
  netWizard.autoConnect(provisioningSsid.c_str(), "");

  if (WiFi.status() == WL_CONNECTED) {
    DAEMON_LOGF("WiFi connected: %s\n", WiFi.localIP().toString().c_str());
    applyWifiStaLinkProfile(true);
    releaseNetWizardHttpIfConnected();
    persistEspNowChannelFromConnectedWifi();
  } else {
    DAEMON_LOGLN("WiFi not connected, configuration AP remains active");
    ensurePortalIfDisconnected();
  }

  applyMdnsName();
  updateWifiChannelCache();
}

void handleWiFi() {
  netWizard.loop();
  applyWifiStaLinkProfile(false);
  handleProvisioningRestart();
  reconnectSavedWifiIfDisconnected();
  ensurePortalIfDisconnected();
  updateWifiChannelCache();

  if (WiFi.status() == WL_CONNECTED) {
    releaseNetWizardHttpIfConnected();
    applyMdnsName();
    persistEspNowChannelFromConnectedWifi();
  }
}

void refreshNetworkIdentity() {
  String deviceName = getDeviceNameLower();
  netWizard.setHostname(deviceName.c_str());
  WiFi.setHostname(deviceName.c_str());
  applyMdnsName();
}
