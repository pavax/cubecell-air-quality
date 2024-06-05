# cubecell-air-quality

Uses the following sensors

* BME680 to measure humidity, temperature, pressure and gas (resistance)
* SCD30 to measure co2, humidity and temperature

Based on the measurements, it calculates a AQI.

# LoraWan Data Paket

```javascript
function parse(payloadHex, port) {
    function hexToBytes(hex) {
        for (var bytes = [], c = 0; c < hex.length; c += 2)
            bytes.push(parseInt(hex.substr(c, 2), 16));
        return bytes;
    }
    var bytes = hexToBytes(payloadHex);
    let msg = {};

    msg.counter = (bytes[0] << 8) | bytes[1];
    msg.battery = (bytes[2] << 8) | bytes[3];

    msg.temperature = (bytes[4] << 24 >> 16 | bytes[5]) / 100;  // signed
    msg.temperature2 = (bytes[6] << 24 >> 16 | bytes[7]) / 100;  // signed

    msg.humidity = ((bytes[8] << 8) | bytes[9]) / 100;
    msg.humidity2 = ((bytes[10] << 8) | bytes[11]) / 100;

    msg.pressure = ((bytes[12] << 24) | (bytes[13] << 16) | (bytes[14] << 8) | bytes[15]) / 100;

    msg.gas = ((bytes[16] << 8) | bytes[17]) / 10;

    msg.aqi = (bytes[18] << 8) | bytes[19];

    msg.co2 = ((bytes[20] << 8) | bytes[21]);
    return msg;
}
```
