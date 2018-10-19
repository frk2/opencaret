---
layout: page
title: Opencaret
subtitle: An open-source/open-data L3 highway autopilot system for modern cars. Initially being perfected on the Kia Soul EV
permalink: index.html
---

## The story
As I inched along on highway 880 (SF Bay Area) on my morning commute down to Santa Clara one day I realized that while level 4/5 urban self driving cars are the future and will forever change the landscape - what would really fix 98% of my driving woes immediately is simply an amazing highway autopilot system. I can easily drive the last few urban miles if the car would just handle the torture and drudgery of droning at 15mph on the highway.

I realize this is not a novel idea. However the devil is in the details. The only fully usable highway autopilot system is Tesla's today with comma.ai's openpilot being a close second. OpenCaret is my attempt to build a reliable and accurate fully open source/open data highway autopilot system that could be installed on any vehicle. 

## Wiki
More information is available on our [Wiki](https://github.com/frk2/opencaret/wiki)

## Why open source?
The accelerated growth of knowledge that we have seen in the software world is driven by the fact that anyone with a computer can contribute. Even though majority of the work that happens in a self driving car startup is software engineering - I realized its very hard for people to start playing in this field without joining a well funded startup or spending enough money to rig up a car.

I want to accelerate this learning by opening up the full implementation (and datasets) to anyone who is interested, including access to the car itself to folks who want to test code live. I hope that this accelerates tinkering and hacking in this space. The cost of a bug could be a life when a self driving car is involved - so instead of wasting human effort on reimplementing the same algorithms again and again - I hope this allows us to come up with novel solutions and advance the state of the art in a meaningful way.

## Is this going to get commercialized?
Not anytime soon. My idea here is to help tinkeres and hackers learn using their own car in the cheapest way possible

## What is working?
The [Wiki](https://github.com/frk2/opencaret/wiki) is kept more updated than this

## How can I help
The car's drive by wire is all hooked up and we are making progress on the autonomy portion. Join our slack or take a look at [Projects](https://github.com/frk2/opencaret/projects) to see whats next

Broadly speaking the main parts to getting this on the road are:

1. Mounting the Radar/Camera/GPS to the car
2. Installing Polysync's OSCC on the car and making it work perfectly
3. Radar based ACC for longitudinal control. MPC for path following
4. Camera based vision and lateral control for lane line detection
5. Mapping and localization to popular highways using RTK-GPS + orbslam2 + depth camera 
6. Centralized real-time highway lane closure inference

You can also join our [Slack Team](https://join.slack.com/t/opencaret/shared_invite/enQtMzU1OTQ2NjY0MTgyLTlhY2JmYTlkYTg4ZGIyNDYzMjFhNWMxNjlmZGFiYTI1MTIyZjA0ODNhMzI5ZDUxYTQ3MWFjYWQ0YzQxZGE1ZjA)

Feel free to email me as well!

## Common sense disclaimer
Messing with your car's throttle/brake/steering sensors is dangerous business and you must totally understand what you are getting into. Always test in areas which are completely safe.

