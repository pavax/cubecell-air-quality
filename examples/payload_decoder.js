/**
 * Payload decoder for CubeCell air quality (Port 2, 26 bytes, big-endian).
 *
 * Drop-in for The Things Stack / ChirpStack v4 (`decodeUplink`).
 * Older stacks: use `Decoder(bytes, port)` below the same way.
 *
 * Example from README (mains, after gas learn):
 *   00 02 10 8A 09 CA 09 74 12 EE 12 AD 00 01 75 9F 48 E8 00 42 06 67 0B 01 48 E8
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
  if (!bytes || bytes.length !== 26) {
    return { errors: ["expected 26 bytes, got " + (bytes ? bytes.length : 0)] };
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

// Optional: encode interval / calibrate / reset downlinks (seconds are big-endian u16).
function encodeDownlink(input) {
  const cmd = input.data || {};
  if (typeof cmd.interval_s === "number") {
    const s = Math.max(0, Math.min(0xffff, Math.round(cmd.interval_s)));
    return { bytes: [(s >> 8) & 0xff, s & 0xff], fPort: 4 };
  }
  if (cmd.calibrate === true) {
    return { bytes: [1], fPort: 5 };
  }
  if (cmd.reset === true) {
    return { bytes: [1], fPort: 9 };
  }
  return { errors: ["unsupported downlink; use interval_s, calibrate, or reset"] };
}
