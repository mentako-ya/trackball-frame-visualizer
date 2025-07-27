# Trackball-frame-visualizer

## Serial device

Print text like below. Visualizer updates screen when it receives `P[484]`. Visualizer starts new frame when it receive `P: start`.

You can find example for PMW3610 sensor with XIAO nRF52840 under `firmware`.

```
P: start
P[0]: 0
P[1]: 10
...
P[484]: 255
```

## Visualizer

Open index.html and connect to serial device.
