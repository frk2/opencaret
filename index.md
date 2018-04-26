---
layout: page
title: Opencaret
subtitle: An open-source/open-data L3 highway autopilot system for modern cars. Initially being perfected on the Kia Soul EV
---

## The story
As I inched along on highway 880 (SF Bay Area) on my morning commute down to Santa clara one day I realized that while level 4/5 urban self driving cars are the future and will forever change the landscape - what would really fix 98% of my driving woes immediately is simply an amazing highway autopilot system. I can easily drive the last few urban miles if the car would just handle the torture and drudgery of droning at 15mph on the highway.

I realize this is not a novel idea. However the devil is in the details. The only fully usable highway autopilot system is Tesla's today with comma.ai's openpilot being a close second. OpenCaret is my attempt to build a reliable and accurate fully opensource/opendata highway autopilot system that could be installed on any vehicle. 

## So whats the idea?
I'm taking the fastest approach to having a working solution which involves getting a Kia Soul EV and putting a drive-by-wire kit on it using [Polysync's OSCC](https://github.com).

Additionally, the plan so far hardware wise is:
- Use a ZED Stereo Camera mounted in the center for depth analyis and 2K video
- Use a off the shelf radar (currently testing a used corolla radar) for logitudinal distance/speed
- Use RTK GPS for a accurate location seed
- A conventional Linux based PC running ubuntu w/NVIDIA GTX 1070 in the trunk. Might shrink this down to a TX2 or couple of TX2 in the future.

Software wise at a high level:
- Use a FCN or other semantic segementation DNN to mark lane lines (similar to comma) and some sort of MPC to follow those lane lines/curvature
- OrbSlam2 using ZED + RTK to localize the car on the road. I understand this requires building maps of current highways. This would be used as a secondary check on the above DNN and for predicting road curvature/drivable region/lane closures.
- Some way of either communicating with other opencaret systems to build in a hive mind of lane/road closures / high disengagements etc.

## Build great documentation along the way
How a self driving car is put together is kind of a mystery for many and a lot of coders I know think they need to get a PhD in order to do anything meaningful. Nothing could be farther than the truth. I plan to document the entire process of getting the car ready in hopes of demystifying it. This would include:

- Detailed step by step instructions of what needs to be ordered and assembled
- Wiring diagrams of cars (starting with the Kia Soul EV) and instructions on how to install the drive by wire kit
- Using off the shelf radars and interfacing with a radar for ACC
- Using off the shelf hardware like a camera, GPS, ultrasonic sensors etc.


In addition I plan to make available any data I use for DNN training purposes as opensource as well, including the DNN model used.

## Why opensource?
The accelerated growth of knowledge that we have seen in the software world is driven by the fact that anyone with a computer can contribute. Even though majority of the work that happens in a self driving car startup is software engineering - I realized its very hard for people to start playing in this field without joining a well funded startup or spending enough money to rig up a car.

I want to accelerate this learning by opening up the full implementation (and datasets) to anyone who is interested, including access to the car itself to folks who want to test code live. I hope that this accelerates tinkering and hacking in this space. The cost of a bug could be a life when a self driving car is involved - so instead of wasting human effort on reimplementing the same algorithms again and again - I hope this allows us to come up with novel solutions and advance the state of the art in a meaningful way.

## All talk and no code!
Yes I do understand that - I'm currently busy on getting the car basics ready so no code to show, but soon! In the meantime [follow me on twitter](https://twitter.com/faraz_r_khan) to get updates on whats going on. I'll be putting up a blog here on GHP soon as well so you can track progress.

## How can I help

By helping wire the car or pushing code! To see what I'm currently working on checkout the team's [Trello Board](https://trello.com/opencaret)

You can also join our [Discord Team](https://discord.gg/yYwb7xK)

Feel free to email me as well!
