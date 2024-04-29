adapted from https://github.com/lmirel/mfc

# mfc - motion feedback controller

this tool uses the native in-game telemetry shared via UDP/TCP from games to control a motion platform.<br>
the following games are supported so far (see clients folder):
- EA WRC (PS5 via USB Proxy) [see for yourself](https://youtu.be/CMydpGEVmhE?si=rpI5bIRiBKAgmEds)<br>
- WRC Generations (PC via its own proxy due to silly local implementation on PC)<br>
- Assetto Corsa (PS4/PS5/PC)<br>
- Codemaster's Dirt Rally 2 (PC and PS4/5 via USB Proxy)<br>
- Project Cars 2 (PS4/PS5/PC)<br>
- GT7 (PS5) [see here](https://youtu.be/65xT8-NaRr0)<br>
<br>
the next thing it does, it uses the USB HID data exchanged between the steering wheel and the gaming console to 'guesstimate' how a motion platform SHOULD move as if it had telemetry available. it does its best and most of the times it manages well. keep in mind however that it guesses that and it relies heavily on the FFB and wheel data to do it.
<br>supported wheels so far:

- Fanatec CSL DD/Elite for PS4/PS5 - best supported
- Logitech G92 for PS4/PS5
- Thrustmaster T300RS for PS4/PS5

# how this works

the MFC SERVER controls the motion platform drivers.<br>
the MFC CLIENTS are programs that receive native telemetry data and sends it to the server (see clients folder for a list of games).<br>
the USB PROXY is used to proxy between the steering wheel and the console to extract USB HID data (requires additonal unexpensive hardware, check subfolder).<br>
the MFC XTRACTOR uses the USB HID data to generate game telemetry.<br>
the two extractors are only needed if the games you play don't provide in-game telemetry via UDP/TCP.<br>
note: the USB XTRACTOR is not used lately in favor of the faster more reliable USB PROXY.
<br>

the end result using the xtractors on a PS4 system looks something like this:<br>
[side view](https://www.youtube.com/watch?v=uBPW2BS_ysU&t=1s) and [back view](https://www.youtube.com/watch?v=jvZpMXiD8k4&t=1s)

# WiKi

- <a href="https://github.com/ehmotion/eh-mfc/wiki">Wiki</a>

<br>Tested on a system using Raspberry 3.
<br><br>
USE AT YOUR OWN RISK and Enjoy!
