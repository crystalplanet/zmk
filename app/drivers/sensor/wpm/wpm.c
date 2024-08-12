
#define WPM_BUFFER_LENGTH 15
#define WPM_INTERVAL 300
#define WPM_SIGMA 15
#define WPM_STROKES_PER_WORD 5

struct wpm_data {
    const struct sensor_trigger *trigger;
    sensor_trigger_handler_t handler;
};

/**
 * Holds the number of keystrokes between subsequent WPM re-calculations.
 * At 300ms intervals, a 15 element buffer covers the last 4.5s.
 */
static uint8_t keystrokes = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/**
 * Indicates where the keystrokes buffer currently ends.
 */
static uint8_t keystrokes_index = 0;

/**
 * Current WPM value.
 */
static uint8_t wpm = 0;

static void wpm_calc(struct k_work *work);

K_WORK_DEFINE(wpm_work, wpm_calc);

static void wpm_tick_handler(struct k_timer *timer) { k_work_submit(&wpm_calc); }

K_TIMER_DEFINE(wpm_tick, wpm_tick_handler, NULL);

static void wpm_calc(struct k_work *work) {
    uint8_t total_keystrokes = 0;

    float averages = 0.0;
    float weights = 0.0;

    for (size_t i = 0; i < WPM_BUFFER_LENGTH; ++i) {
        float distance = keystrokes_index - i + (i > keystrokes_index ? WPM_BUFFER_LENGTH : 0);
        float weight = exp(-((distance * distance) / (2 * WPM_SIGMA)));

        weights += weight;
        averages += weight * keystrokes[i];

        total_keystrokes += keystrokes[i];
    }

    uint8_t new_wpm = ((averages / weights) * ((1000 * 60) / WPM_INTERVAL)) / WPM_STROKES_PER_WORD;

    if (++keystrokes_index == WPM_CALC_BUFFER_LENGTH) {
        keystrokes_index = 0;
    }

    keystrokes[keystrokes_index] = 0;

    if (total_keystrokes == 0) {
        k_timer_stop(&wpm_tick);
    }
}

static int wpm_trigger_set(const struct device *dev, const struct sensor_trigger *trigger,
                           sensor_trigger_handler_t handler) {
    struct wpm_data *data = dev->data;

    data->trigger = trigger;
    data->handler = handler;

    return 0;
}

#define WPM_SENSOR_DEVICE(idx)
\
