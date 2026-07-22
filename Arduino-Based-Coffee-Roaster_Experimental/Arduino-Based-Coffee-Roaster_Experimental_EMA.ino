/*
MAX6675
      │
      ▼
NaN/Open detection
      │
      ▼
Median of 3
      │
      ▼
Rate-of-change limiter
      │
      ▼
EMA (15/16)
      │
      ▼
Hysteresis
      │
      ▼
Modbus register
*/

#define TEMP_SAMPLE_INTERVAL_MS   250

#define TEMP_MIN_VALID            100       // 1.00°C
#define TEMP_MAX_VALID            30000   // 300.00°C

#define TEMP_MAX_STEP             1000     // 10.00°C/sample

#define TEMP_HYSTERESIS           25      // 0.25°C (one LSB)


//=============================================================================
// Temperature filter state
//=============================================================================

static uint16_t lastRawTemp = 0;
static bool hasRawTemp = false;

static uint16_t medianBuffer[3];
static uint8_t medianIndex = 0;
static bool medianReady = false;

static uint32_t emaTemp = 0;              // x100 °C
static bool emaInitialized = false;

static uint16_t lastPublishedTemp = 0;

static uint8_t invalidCount = 0;

static uint32_t tempTime = 0;

// Median helper
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }

    return b;
}

// Temperature reading

void tempReading()
{
    uint32_t now = millis();

    if ((now - tempTime) < TEMP_SAMPLE_INTERVAL_MS)
        return;

    tempTime += TEMP_SAMPLE_INTERVAL_MS;

    //---------------------------------------------------------------------
    // Read MAX6675
    //---------------------------------------------------------------------

    float temperatureC = thermocoupleBT.readCelsius();

    //---------------------------------------------------------------------
    // NaN / Open Thermocouple
    //---------------------------------------------------------------------

    if (isnan(temperatureC))
    {
        if (++invalidCount >= 5)
        {
            // Optional error indication
            // au16data[2] = 0xFFFF;
        }

        return;
    }

    invalidCount = 0;

    //---------------------------------------------------------------------
    // Convert to centi-degrees
    //---------------------------------------------------------------------

    uint16_t temperature =
        (uint16_t)(temperatureC * 100.0f + 0.5f);

    //---------------------------------------------------------------------
    // Absolute limits
    //---------------------------------------------------------------------

    if (temperature < TEMP_MIN_VALID ||
        temperature > TEMP_MAX_VALID)
        return;

    //---------------------------------------------------------------------
    // Median-of-3 filter
    //---------------------------------------------------------------------

    medianBuffer[medianIndex] = temperature;

    medianIndex++;

    if (medianIndex >= 3)
    {
        medianIndex = 0;
        medianReady = true;
    }

    if (!medianReady)
        return;

    temperature = median3(
        medianBuffer[0],
        medianBuffer[1],
        medianBuffer[2]);

    //---------------------------------------------------------------------
    // Rate limiter
    //---------------------------------------------------------------------

    if (hasRawTemp)
    {
        int16_t delta =
            (int16_t)temperature - (int16_t)lastRawTemp;

        if (delta > TEMP_MAX_STEP)
            temperature = lastRawTemp + TEMP_MAX_STEP;

        else if (delta < -TEMP_MAX_STEP)
            temperature = lastRawTemp - TEMP_MAX_STEP;
    }

    lastRawTemp = temperature;
    hasRawTemp = true;

    //---------------------------------------------------------------------
    // EMA (15/16)
    //---------------------------------------------------------------------

    if (!emaInitialized)
    {
        emaTemp = temperature;
        emaInitialized = true;
    }
    else
    {
        emaTemp = ((emaTemp * 15UL) + temperature) / 16UL;
    }

    uint16_t filteredTemp = (uint16_t)emaTemp;

    //---------------------------------------------------------------------
    // Hysteresis
    //---------------------------------------------------------------------

    if (abs((int16_t)filteredTemp -
            (int16_t)lastPublishedTemp) >= TEMP_HYSTERESIS)
    {
        lastPublishedTemp = filteredTemp;
        au16data[2] = filteredTemp;
    }
}
