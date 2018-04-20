---
layout: default
---

## The story
As I inched along on highway 880 (SF Bay Area) on my morning commute down to Santa clara one day I realized that while level 4/5 urban self driving cars are the future and will forever change the landscape - what would really fix 98% of my driving woes immediately is simply an amazing highway autopilot system. I can easily drive the last few urban miles if the car would just handle the torture and drudgery of droning at 15mph on the highway.

I realize this is not a novel idea. However the devil is in the details. The only fully usable highway autopilot system is Tesla's today with comma.ai's openpilot being a close second. OpenCaret is my attempt to build a reliable and accurate fully opensource/opendata highway autopilot system that could be installed on any vehicle. 

## So whats the idea?
I'm taking the fastest approach to having a working solution which involves getting a Kia Soul EV and putting a drive-by-wire kit on it using [Polysync's OSCC](https://github.com).

I plan to document the entire process of getting the car ready as well. This would include:

- Detailed step by step instructions of what needs to be ordered and assembled
- Wiring diagrams of cars (starting with the Kia Soul EV) and instructions on how to install the drive by wire kit
- Using off the shelf radars and interfacing with a radar for ACC
- Using off the shelf hardware like a camera, GPS, ultrasonic sensors etc.


In addition I plan to make available any data I use for DNN training purposes as opensource as well, including the DNN model used.

## Why opensource?
The accelerated growth of knowledge that we have seen in the software world is driven by the fact that anyone with a computer can contribute. Even though majority of the work that happens in a self driving car startup is software engineering - I realized its very hard for people to start playing in this field without joining a well funded startup or spending enough money to rig up a car.

I want to accelerate this learning by opening up the full implementation (and datasets) to anyone who is interested, including access to the car itself to folks who want to test code live. I hope that this accelerates tinkering and hacking in this space. The cost of a bug could be a life when a self driving car is involved - so instead of wasting human effort on reimplementing the same algorithms again and again - I hope this allows us to come up with novel solutions and advance the state of the art in a meaningful way.


## How can I help

https://trello.com/opencaret
