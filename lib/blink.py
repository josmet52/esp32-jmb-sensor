import machine
import utime

INTERNAL_BLUE_LED_PIN = 2
internal_blue_led = machine.Pin(INTERNAL_BLUE_LED_PIN, machine.Pin.OUT)

def blink_internal_blue_led(t_on_ms, t_off_ms, t_pause_ms, n_repeat):
    
    for n in range(n_repeat):
        internal_blue_led.on()
        utime.sleep_ms(t_on_ms)
        internal_blue_led.off()
        utime.sleep_ms(t_off_ms)
    utime.sleep_ms(t_pause_ms)
    
if __name__ == '__main__':
    blink_internal_blue_led(500, 100, 1000, 5)
