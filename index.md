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

## How will this ever be sustainable?
The primary focus for the first 9-12 months isn't to make money, but to deliver on a realiable autopilot system that has a active community around it. Im personally financially secure enough to make sure this becomes a reality.

I do not think selling this as a retro-fit kit is going to be sustainable. The future idea is to partner up with a tier 1 or another automotive supplier to provide a robust car specific drive by wire interface upon which our software could be implemented. The entire package consisting of the DBW/Radar/Camera/GPS/Compute would then need to be professionally installed at your local Dealership/Walmart/Autozone/Pepboys/car repair shop. These guys know how to make amazing hardware and how to interface with cars in a bullet-proof way. We just need to make the software awesome. Sounds far out and hard but thats why these are called hard problems to solve :)

## All talk and no code!
Yes I do understand that - I'm currently busy on getting the car basics ready so no code to show, but soon! In the meantime [follow me on twitter](https://twitter.com/faraz_r_khan) to get updates on whats going on. I'm also going to keep the [Blog](https://medium.com/opencaret) as up-to-date as possible.


## How can I help
By helping wire the car or pushing code! To see what I'm currently working on checkout the team's [Trello Board](https://trello.com/opencaret)

Broadly speaking the main parts to getting this on the road are:

1. Mounting the Radar/Camera/GPS to the car
2. Installing Polysync's OSCC on the car and making it work perfectly
3. Camera based vision and lateral control for lane line detection
4. Radar based ACC for longitudinal control. MPC for path following
5. Mapping and localization to popular highways using RTK-GPS + orbslam2 + depth camera 
6. Centralized real-time highway lane closure inference

You can also join our [Slack Team](https://join.slack.com/t/opencaret/shared_invite/enQtMzU1OTQ2NjY0MTgyLTlhY2JmYTlkYTg4ZGIyNDYzMjFhNWMxNjlmZGFiYTI1MTIyZjA0ODNhMzI5ZDUxYTQ3MWFjYWQ0YzQxZGE1ZjA)

Feel free to email me as well!

## Common sense disclaimer
Messing with your car's throttle/brake/steering sensors is dangerous business and you must totally understand what you are getting into. Always test in areas which are completely safe.

