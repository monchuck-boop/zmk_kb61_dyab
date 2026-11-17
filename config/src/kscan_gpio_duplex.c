#define DT_DRV_COMPAT zmk_kscan_gpio_duplex

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct duplex_key_state {
    bool pressed;
    uint8_t counter;
};

struct kscan_duplex_config {
    const struct gpio_dt_spec *rows;
    size_t row_count;
    const struct gpio_dt_spec *cols;
    size_t col_count;
    int32_t debounce_press_ms;
    int32_t debounce_release_ms;
    int32_t poll_period_ms;
};

struct kscan_duplex_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    struct duplex_key_state *states;
};

static void kscan_duplex_discharge_and_idle(const struct kscan_duplex_config *cfg) {
    /* 残留電荷を強制的にLOWへ逃がしてからINPUTに戻す（ゴースト防止） */
    for (size_t r = 0; r < cfg->row_count; r++) {
        gpio_pin_configure_dt(&cfg->rows[r], GPIO_OUTPUT_INACTIVE);
    }
    for (size_t c = 0; c < cfg->col_count; c++) {
        gpio_pin_configure_dt(&cfg->cols[c], GPIO_OUTPUT_INACTIVE);
    }
    k_busy_wait(15);

    for (size_t r = 0; r < cfg->row_count; r++) {
        gpio_pin_configure_dt(&cfg->rows[r], GPIO_INPUT);
    }
    for (size_t c = 0; c < cfg->col_count; c++) {
        gpio_pin_configure_dt(&cfg->cols[c], GPIO_INPUT);
    }
}

static void kscan_duplex_update_state(const struct device *dev, size_t r, size_t c, bool raw_pressed) {
    const struct kscan_duplex_config *cfg = dev->config;
    struct kscan_duplex_data *data = dev->data;
    const size_t total_cols = cfg->col_count * 2;
    struct duplex_key_state *st = &data->states[r * total_cols + c];

    uint8_t threshold = (uint8_t)((raw_pressed ? cfg->debounce_press_ms : cfg->debounce_release_ms) /
                                  (cfg->poll_period_ms > 0 ? cfg->poll_period_ms : 1));
    if (threshold < 1) {
        threshold = 1;
    }

    if (raw_pressed != st->pressed) {
        st->counter++;
        if (st->counter >= threshold) {
            st->pressed = raw_pressed;
            st->counter = 0;
            if (data->callback) {
                data->callback(dev, r, c, st->pressed);
            }
        }
    } else {
        st->counter = 0;
    }
}

static void kscan_duplex_read(const struct device *dev) {
    const struct kscan_duplex_config *cfg = dev->config;

    /* Phase 1: row2col (論理列 0 .. col_count - 1 : 左側の列) */
    for (size_t c = 0; c < cfg->col_count; c++) {
        gpio_pin_configure_dt(&cfg->cols[c], GPIO_INPUT | GPIO_PULL_DOWN);
    }
    for (size_t r = 0; r < cfg->row_count; r++) {
        gpio_pin_configure_dt(&cfg->rows[r], GPIO_OUTPUT_ACTIVE);
        k_busy_wait(30);
        for (size_t c = 0; c < cfg->col_count; c++) {
            bool pressed = gpio_pin_get_dt(&cfg->cols[c]) > 0;
            kscan_duplex_update_state(dev, r, c, pressed);
        }
        gpio_pin_configure_dt(&cfg->rows[r], GPIO_OUTPUT_INACTIVE);
        k_busy_wait(10);
        gpio_pin_configure_dt(&cfg->rows[r], GPIO_INPUT);
    }

    kscan_duplex_discharge_and_idle(cfg);

    /* Phase 2: col2row (論理列 col_count .. 2*col_count - 1 : 右側の列) */
    for (size_t r = 0; r < cfg->row_count; r++) {
        gpio_pin_configure_dt(&cfg->rows[r], GPIO_INPUT | GPIO_PULL_DOWN);
    }
    for (size_t c = 0; c < cfg->col_count; c++) {
        gpio_pin_configure_dt(&cfg->cols[c], GPIO_OUTPUT_ACTIVE);
        k_busy_wait(30);
        for (size_t r = 0; r < cfg->row_count; r++) {
            bool pressed = gpio_pin_get_dt(&cfg->rows[r]) > 0;
            kscan_duplex_update_state(dev, r, cfg->col_count + c, pressed);
        }
        gpio_pin_configure_dt(&cfg->cols[c], GPIO_OUTPUT_INACTIVE);
        k_busy_wait(10);
        gpio_pin_configure_dt(&cfg->cols[c], GPIO_INPUT);
    }

    kscan_duplex_discharge_and_idle(cfg);
}

static void kscan_duplex_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct kscan_duplex_data *data = CONTAINER_OF(dwork, struct kscan_duplex_data, work);
    const struct kscan_duplex_config *cfg = data->dev->config;

    kscan_duplex_read(data->dev);
    k_work_schedule(&data->work, K_MSEC(cfg->poll_period_ms));
}

static int kscan_duplex_configure(const struct device *dev, kscan_callback_t callback) {
    struct kscan_duplex_data *data = dev->data;
    if (!callback) {
        return -EINVAL;
    }
    data->callback = callback;
    return 0;
}

static int kscan_duplex_enable(const struct device *dev) {
    struct kscan_duplex_data *data = dev->data;
    const struct kscan_duplex_config *cfg = dev->config;
    k_work_schedule(&data->work, K_MSEC(cfg->poll_period_ms));
    return 0;
}

static int kscan_duplex_disable(const struct device *dev) {
    struct kscan_duplex_data *data = dev->data;
    k_work_cancel_delayable(&data->work);
    return 0;
}

static int kscan_duplex_init(const struct device *dev) {
    struct kscan_duplex_data *data = dev->data;
    const struct kscan_duplex_config *cfg = dev->config;

    data->dev = dev;
    kscan_duplex_discharge_and_idle(cfg);
    k_work_init_delayable(&data->work, kscan_duplex_work_handler);
    return 0;
}

static const struct kscan_driver_api kscan_duplex_api = {
    .config = kscan_duplex_configure,
    .enable_callback = kscan_duplex_enable,
    .disable_callback = kscan_duplex_disable,
};

#define KSCAN_DUPLEX_INIT(n)                                                                       \
    static const struct gpio_dt_spec kscan_duplex_rows_##n[] = {                                   \
        DT_INST_FOREACH_PROP_ELEM_SEP(n, row_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};               \
    static const struct gpio_dt_spec kscan_duplex_cols_##n[] = {                                   \
        DT_INST_FOREACH_PROP_ELEM_SEP(n, col_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};               \
    static struct duplex_key_state                                                                 \
        kscan_duplex_states_##n[DT_INST_PROP_LEN(n, row_gpios) * DT_INST_PROP_LEN(n, col_gpios) *  \
                                2];                                                                \
    static const struct kscan_duplex_config kscan_duplex_config_##n = {                            \
        .rows = kscan_duplex_rows_##n,                                                             \
        .row_count = ARRAY_SIZE(kscan_duplex_rows_##n),                                            \
        .cols = kscan_duplex_cols_##n,                                                             \
        .col_count = ARRAY_SIZE(kscan_duplex_cols_##n),                                            \
        .debounce_press_ms = DT_INST_PROP(n, debounce_press_ms),                                   \
        .debounce_release_ms = DT_INST_PROP(n, debounce_release_ms),                               \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                         \
    };                                                                                             \
    static struct kscan_duplex_data kscan_duplex_data_##n = {                                      \
        .states = kscan_duplex_states_##n,                                                         \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, kscan_duplex_init, NULL, &kscan_duplex_data_##n,                      \
                          &kscan_duplex_config_##n, POST_KERNEL,                                   \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &kscan_duplex_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_DUPLEX_INIT)
