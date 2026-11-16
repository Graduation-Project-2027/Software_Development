# Stack light and buzzer request for the body controller (CAN frame BODY_CMD, 0x020).
# Publish it at 1 Hz or faster: the CAN bridge falls back to LIGHT_IDLE / BUZZER_OFF after
# indicator_timeout. The bridge shows LIGHT_MOVING by itself while the wheels are commanded
# and LIGHT_FAULT on a drive fault. A latched e-stop always overrides the request
# (red blinking, handled by the controller without the Pi).

uint8 LIGHT_IDLE = 0     # green steady
uint8 LIGHT_MOVING = 1   # green steady + amber blinking
uint8 LIGHT_PAUSED = 2   # amber steady
uint8 LIGHT_FAULT = 3    # red steady

uint8 BUZZER_OFF = 0
uint8 BUZZER_SLOW = 1    # one short beep per second (travel alarm)
uint8 BUZZER_FAST = 2    # 5 Hz beeping (lifting, warning)

uint8 light
uint8 buzzer
