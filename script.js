const THEME_KEY = "espd-flasher-light-mode";
const FLASH_MODE_KEY = "espd-flasher-mode";

const FLASH_MODES = {
  product: {
    manifest: "manifest-product.json",
    fallbackFirmware: "firmware-product.bin"
  },
  estop: {
    manifest: "manifest-estop.json",
    fallbackFirmware: "firmware-estop.bin"
  }
};

const ui = {
  themeBtn: document.getElementById("themeBtn"),
  refreshBtn: document.getElementById("refreshBtn"),
  metaStatusChipEl: document.getElementById("metaStatusChip"),
  metaStatusEl: document.getElementById("metaStatus"),
  versionEl: document.getElementById("version"),
  timestampEl: document.getElementById("timestamp"),
  firmwareSizeEl: document.getElementById("firmware-size"),
  firmwareMd5El: document.getElementById("firmware-md5"),
  installBtn: document.getElementById("installBtn"),
  modeProductBtn: document.getElementById("modeProductBtn"),
  modeEstopBtn: document.getElementById("modeEstopBtn")
};

const state = {
  latestData: null,
  mode: loadSavedMode()
};

document.addEventListener("DOMContentLoaded", () => {
  const prefersLight = localStorage.getItem(THEME_KEY) === "true";
  if (prefersLight) {
    document.body.classList.add("light-mode");
  }
  updateThemeButton();

  ui.themeBtn.addEventListener("click", toggleTheme);
  ui.refreshBtn.addEventListener("click", loadLatestVersion);
  ui.modeProductBtn.addEventListener("click", () => selectMode("product", true));
  ui.modeEstopBtn.addEventListener("click", () => selectMode("estop", true));

  selectMode(state.mode, false);
  loadLatestVersion();
});

function loadSavedMode() {
  const saved = localStorage.getItem(FLASH_MODE_KEY);
  return saved && FLASH_MODES[saved] ? saved : "product";
}

function toggleTheme() {
  document.body.classList.toggle("light-mode");
  localStorage.setItem(THEME_KEY, document.body.classList.contains("light-mode") ? "true" : "false");
  updateThemeButton();
}

function updateThemeButton() {
  const isLight = document.body.classList.contains("light-mode");
  ui.themeBtn.textContent = isLight ? "Dark" : "Light";
  ui.themeBtn.setAttribute("aria-label", isLight ? "Switch to dark mode" : "Switch to light mode");
}

async function loadLatestVersion() {
  setLoadingState();

  try {
    const response = await fetch("latest.json", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`Failed to fetch latest.json: ${response.status}`);
    }

    state.latestData = await response.json();
    renderFirmwareInfo();
    setMetaStatus("Online");
  } catch (error) {
    console.error("Failed to load firmware metadata:", error);
    state.latestData = null;
    ui.versionEl.textContent = "Error";
    ui.timestampEl.textContent = "Failed to load";
    ui.firmwareSizeEl.textContent = "Failed to load";
    ui.firmwareMd5El.textContent = "Failed to load";
    setMetaStatus("Offline");
  } finally {
    ui.refreshBtn.disabled = false;
  }
}

function setLoadingState() {
  ui.refreshBtn.disabled = true;
  ui.versionEl.textContent = "Loading...";
  ui.timestampEl.textContent = "Loading...";
  ui.firmwareSizeEl.textContent = "Loading...";
  ui.firmwareMd5El.textContent = "Loading...";
  setMetaStatus("Syncing");
}

function selectMode(mode, persist) {
  if (!FLASH_MODES[mode]) {
    return;
  }
  state.mode = mode;
  if (persist) {
    localStorage.setItem(FLASH_MODE_KEY, mode);
  }
  updateModeUi();
  updateManifestTarget();
  renderFirmwareInfo();
}

function updateModeUi() {
  const isProduct = state.mode === "product";
  ui.modeProductBtn.classList.toggle("is-active", isProduct);
  ui.modeEstopBtn.classList.toggle("is-active", !isProduct);
  ui.modeProductBtn.setAttribute("aria-pressed", isProduct ? "true" : "false");
  ui.modeEstopBtn.setAttribute("aria-pressed", isProduct ? "false" : "true");
}

function updateManifestTarget() {
  const availableModes = getAvailableModes(state.latestData);
  const effectiveMode = availableModes.includes(state.mode) ? state.mode : availableModes[0];
  const targetManifest = FLASH_MODES[effectiveMode]?.manifest || "manifest.json";
  ui.installBtn.setAttribute("manifest", targetManifest);
}

function getAvailableModes(data) {
  if (!data || !data.files) {
    return ["product", "estop"];
  }

  const available = [];
  if (resolveFirmwareName(data, "product")) {
    available.push("product");
  }
  if (resolveFirmwareName(data, "estop")) {
    available.push("estop");
  }
  return available.length > 0 ? available : ["product"];
}

function renderFirmwareInfo() {
  if (!state.latestData) {
    return;
  }

  const availableModes = getAvailableModes(state.latestData);
  if (!availableModes.includes(state.mode)) {
    state.mode = availableModes[0];
    localStorage.setItem(FLASH_MODE_KEY, state.mode);
    updateModeUi();
  }

  ui.modeProductBtn.disabled = !availableModes.includes("product");
  ui.modeEstopBtn.disabled = !availableModes.includes("estop");
  updateManifestTarget();

  const data = state.latestData;
  const files = data.files || {};
  const firmwareName = resolveFirmwareName(data, state.mode);
  const firmware = firmwareName ? files[firmwareName] : null;
  const version = data.version || "Unknown";

  ui.versionEl.textContent = version;
  ui.timestampEl.textContent = formatDate(data.timestamp) || "Unknown";
  ui.firmwareSizeEl.textContent = firmware ? formatBytes(firmware.size) : "Unavailable";
  ui.firmwareMd5El.textContent = firmware?.md5 || "Unavailable";
}

function resolveFirmwareName(data, mode) {
  const files = data?.files || {};
  const variantFirmware = data?.variants?.[mode]?.firmware;
  if (variantFirmware && files[variantFirmware]) {
    return variantFirmware;
  }

  const modeFallback = FLASH_MODES[mode]?.fallbackFirmware;
  if (modeFallback && files[modeFallback]) {
    return modeFallback;
  }

  return null;
}

function setMetaStatus(status) {
  ui.metaStatusEl.textContent = status;
  ui.metaStatusEl.classList.remove("is-ready", "is-syncing", "is-offline");
  ui.metaStatusChipEl.classList.remove("is-ready", "is-syncing", "is-offline");
  if (status === "Ready" || status === "Online") {
    ui.metaStatusEl.classList.add("is-ready");
    ui.metaStatusChipEl.classList.add("is-ready");
  } else if (status === "Syncing") {
    ui.metaStatusEl.classList.add("is-syncing");
    ui.metaStatusChipEl.classList.add("is-syncing");
  } else if (status === "Offline") {
    ui.metaStatusEl.classList.add("is-offline");
    ui.metaStatusChipEl.classList.add("is-offline");
  }
}

function formatBytes(bytes) {
  if (!Number.isFinite(bytes) || bytes <= 0) {
    return "0 B";
  }
  const units = ["B", "KB", "MB", "GB"];
  const index = Math.min(Math.floor(Math.log(bytes) / Math.log(1024)), units.length - 1);
  const value = bytes / Math.pow(1024, index);
  return `${Math.round(value * 100) / 100} ${units[index]}`;
}

function formatDate(isoString) {
  if (!isoString) {
    return null;
  }
  const date = new Date(isoString);
  if (Number.isNaN(date.getTime())) {
    return null;
  }
  const y = date.getFullYear();
  const m = String(date.getMonth() + 1).padStart(2, "0");
  const d = String(date.getDate()).padStart(2, "0");
  const h = String(date.getHours()).padStart(2, "0");
  const min = String(date.getMinutes()).padStart(2, "0");
  const s = String(date.getSeconds()).padStart(2, "0");
  return `${y}-${m}-${d} ${h}:${min}:${s}`;
}
