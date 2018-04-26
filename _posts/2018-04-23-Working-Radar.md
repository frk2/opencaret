---
layout: post
title: The radar is alive!
image: /img/hello_world.jpeg
published: true
---

Since im waiting on the macrofab order for Polysync OSCC to come through I thought I'll try and get the radar to actually spit out real information. I thought this would be as easy as connecting the radar and deciphering its CAN messages. 

Ofcourse it was way harder than that! Turns out the radar has to be turned ON by a sequence of CAN bus messages from the DSU. Comma.ai's openpilot handles this situation so I started going through all there code and sending all possible messages I could think of. In the end I gave up and realized I can simply sniff out all the messages that openpilot sends on the CAN bus to the radar! This actually worked! I was able to separate out the messages openpilot was sending and upon sending those from my python script the radar came to life!

![The radar on my desk connected to two CAN bus USB adaptors]({{site.baseurl}}/img/IMG_20180425_180230.jpg)

If you want to do this yourself- take a look at my [Toyota Radar Can control project](https://github.com/frk2/toyoyta_radar_control_can)

Now to actually mount this thing on the car