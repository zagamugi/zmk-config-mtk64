#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>

#define ADV_TOGGLE_MS 100
#define LAYER_STEP_MS 400

enum led_mode {
    LED_MODE_OFF,
    LED_MODE_ADVERTISING,
    LED_MODE_CONNECTED,
};

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
BUILD_ASSERT(DT_NODE_EXISTS(DT_ALIAS(led0)), "led0 alias is required for dongle status LED");

static enum led_mode current_mode = LED_MODE_OFF;
static atomic_t layer_blink_active;
static atomic_t layer_blink_count;
static bool led_state;

static void led_set(bool on) {
    if (!device_is_ready(led.port)) {
        return;
    }
    gpio_pin_set_dt(&led, on);
    led_state = on;
}

static void adv_timer_handler(struct k_timer *timer_id) {
    ARG_UNUSED(timer_id);
    led_set(!led_state);
}

K_TIMER_DEFINE(adv_timer, adv_timer_handler, NULL);
K_SEM_DEFINE(layer_blink_sem, 0, 1);

static enum led_mode compute_mode(void) {
    if (zmk_ble_active_profile_is_connected()) {
        return LED_MODE_CONNECTED;
    }
    if (zmk_ble_active_profile_is_open()) {
        return LED_MODE_ADVERTISING;
    }
    return LED_MODE_OFF;
}

static void apply_mode(void) {
    if (atomic_get(&layer_blink_active)) {
        if (current_mode != LED_MODE_ADVERTISING) {
            k_timer_stop(&adv_timer);
        }
        return;
    }

    switch (current_mode) {
    case LED_MODE_ADVERTISING:
        k_timer_start(&adv_timer, K_NO_WAIT, K_MSEC(ADV_TOGGLE_MS));
        break;
    case LED_MODE_CONNECTED:
        k_timer_stop(&adv_timer);
        led_set(true);
        break;
    default:
        k_timer_stop(&adv_timer);
        led_set(false);
        break;
    }
}

static void layer_blink_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        k_sem_take(&layer_blink_sem, K_FOREVER);

        uint8_t count = (uint8_t)atomic_get(&layer_blink_count);
        if (count == 0) {
            continue;
        }

        atomic_set(&layer_blink_active, 1);
        k_timer_stop(&adv_timer);

        for (uint8_t i = 0; i < count; i++) {
            led_set(false);
            k_sleep(K_MSEC(LAYER_STEP_MS));
            led_set(true);
            k_sleep(K_MSEC(LAYER_STEP_MS));
        }

        atomic_set(&layer_blink_active, 0);
        apply_mode();
    }
}

K_THREAD_DEFINE(layer_blink_tid, 512, layer_blink_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

static int led_conn_listener_cb(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    current_mode = compute_mode();
    apply_mode();
    return 0;
}

ZMK_LISTENER(led_conn_listener, led_conn_listener_cb);
ZMK_SUBSCRIPTION(led_conn_listener, zmk_ble_active_profile_changed);

static int led_layer_listener_cb(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);

    if (ev == NULL || !ev->state) {
        return 0;
    }

    if (current_mode != LED_MODE_CONNECTED) {
        return 0;
    }

    uint8_t layer = zmk_keymap_highest_layer_active();
    if (layer == 0) {
        return 0;
    }

    atomic_set(&layer_blink_count, layer);
    k_sem_give(&layer_blink_sem);
    return 0;
}

ZMK_LISTENER(led_layer_listener, led_layer_listener_cb);
ZMK_SUBSCRIPTION(led_layer_listener, zmk_layer_state_changed);

static int mtk64_dongle_led_init(void) {
    if (!device_is_ready(led.port)) {
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (err) {
        return err;
    }

    /* Quick boot blink for simple startup confirmation */
    for (int i = 0; i < 3; i++) {
        led_set(true);
        k_sleep(K_MSEC(80));
        led_set(false);
        k_sleep(K_MSEC(80));
    }

    current_mode = compute_mode();
    apply_mode();
    return 0;
}

SYS_INIT(mtk64_dongle_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);