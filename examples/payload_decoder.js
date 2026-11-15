/**
 * Payload decoder for CubeCell air quality (Port 2, 27 bytes, big-endian).
 *
 * Drop-in for The Things Stack / ChirpStack v4 (`decodeUplink`).
 * Older stacks: use `Decoder(bytes, port)` below the same way.
 *
 * Numeric sensor fields are window aggregates since the previous uplink:
 * temperatures/pressure = mean, humidity = sample with worst humidity score,
 * CO2 = max, gas = min, AQI = min. Status and gas baseline are from the
 * current cycle at send time. Battery mV is the last reading in the window.
 * sample_count is the number of measure cycles in the window (max 255).
 *
 * Example from README (mains, after gas learn):
 *   00 02 10 8A 09 CA 09 74 12 EE 12 AD 00 01 75 9F 48 E8 00 42 06 67 0B 01 48 E8 01
 */

function u16(bytes, i) {
  return (bytes[i] << 8) | bytes[i + 1];
}

function i16(bytes, i) {
  const v = u16(bytes, i);
  return v > 0x7fff ? v - 0x10000 : v;
}

function u32(bytes, i) {
  return ((bytes[i] << 24) >>> 0) + (bytes[i + 1] << 16) + (bytes[i + 2] << 8) + bytes[i + 3];
}

function sensorStatusLabel(code) {
  if (code === 1) return "ok";
  if (code === 0) return "missing";
  if (code === 9) return "timeout";
  return "unknown";
}

function decodePayload(bytes, port) {
  if (port !== 2) {
    return { errors: ["unexpected port " + port] };
  }
  if (!bytes || bytes.length !== 27) {
    return { errors: ["expected 27 bytes, got " + (bytes ? bytes.length : 0)] };
  }

  const status = bytes[22];
  const bmeStatus = Math.floor(status / 10);
  const scdStatus = status % 10;
  const gasRaw = u16(bytes, 16);
  const gasBaselineRaw = u16(bytes, 24);
  const pressurePa = u32(bytes, 12);

  return {
    data: {
      counter: u16(bytes, 0),
      battery_mV: u16(bytes, 2),
      temperature_bme_C: i16(bytes, 4) / 100,
      temperature_scd_C: i16(bytes, 6) / 100,
      humidity_bme_pct: u16(bytes, 8) / 100,
      humidity_scd_pct: u16(bytes, 10) / 100,
      pressure_Pa: pressurePa,
      pressure_hPa: pressurePa / 100,
      gas_ohm: gasRaw * 10,
      gas_baseline_ohm: gasBaselineRaw * 10,
      air_quality: u16(bytes, 18),
      co2_ppm: u16(bytes, 20),
      status: status,
      bme_status: bmeStatus,
      scd_status: scdStatus,
      bme_ok: bmeStatus === 1,
      scd_ok: scdStatus === 1,
      bme_status_label: sensorStatusLabel(bmeStatus),
      scd_status_label: sensorStatusLabel(scdStatus),
      mains: bytes[23] === 1,
      sample_count: bytes[26],
    },
  };
}

// The Things Stack / ChirpStack v4
function decodeUplink(input) {
  return decodePayload(input.bytes, input.fPort);
}

// ChirpStack v3 / older TTN
function Decoder(bytes, port) {
  const decoded = decodePayload(bytes, port);
  return decoded.data || { error: (decoded.errors || []).join("; ") };
}

function clampU16(value) {
  return Math.max(0, Math.min(0xffff, Math.round(value)));
}

function beU16(value) {
  const v = clampU16(value);
  return [(v >> 8) & 0xff, v & 0xff];
}

function pickNumber() {
  for (let i = 0; i < arguments.length; i++) {
    if (typeof arguments[i] === "number") {
      return arguments[i];
    }
  }
  return null;
}

// Optional: encode interval / calibrate / reset downlinks (seconds are big-endian u16).
// Port 4 (8 bytes):
//   battery_mode_measure_interval_s + battery_mode_tx_interval_s +
//   mains_mode_measure_interval_s + mains_mode_tx_interval_s
// Aliases: battery_mode_interval_s / mains_mode_interval_s set measure=tx;
//          interval_s sets all four.
function encodeDownlink(input) {
  const cmd = input.data || {};
  const batteryMeasure = pickNumber(
    cmd.battery_mode_measure_interval_s,
    cmd.battery_mode_interval_s,
    cmd.battery_interval_s,
    cmd.interval_s
  );
  const batteryTx = pickNumber(
    cmd.battery_mode_tx_interval_s,
    cmd.battery_mode_interval_s,
    cmd.battery_interval_s,
    cmd.interval_s
  );
  const mainsMeasure = pickNumber(
    cmd.mains_mode_measure_interval_s,
    cmd.mains_measure_interval_s,
    cmd.mains_mode_interval_s,
    cmd.mains_interval_s,
    cmd.interval_s
  );
  const mainsTx = pickNumber(
    cmd.mains_mode_tx_interval_s,
    cmd.mains_tx_interval_s,
    cmd.mains_mode_interval_s,
    cmd.mains_interval_s,
    cmd.interval_s
  );

  if (
    batteryMeasure != null &&
    batteryTx != null &&
    mainsMeasure != null &&
    mainsTx != null
  ) {
    return {
      bytes: beU16(batteryMeasure)
        .concat(beU16(batteryTx), beU16(mainsMeasure), beU16(mainsTx)),
      fPort: 4,
    };
  }
  if (cmd.calibrate === true) {
    return { bytes: [1], fPort: 5 };
  }
  if (cmd.reset === true) {
    return { bytes: [1], fPort: 9 };
  }
  return {
    errors: [
      "unsupported downlink; use battery_mode_measure_interval_s+battery_mode_tx_interval_s+mains_mode_measure_interval_s+mains_mode_tx_interval_s, interval_s, calibrate, or reset",
    ],
  };
}
