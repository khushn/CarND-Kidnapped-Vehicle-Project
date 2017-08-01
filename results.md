### Summary of Observations and learnings

1. Input data: 
   A big hurdle is the noise in the input data. One input yaw_rate value is 62.697. As soon as its recieved the yaw error goes out of range. 
   I suspect this could be simulator bug. I am using the Ubuntu version v1.45)
   <p>
   <a href= "error_occurs_on_eroneous_input.mp4"> Video showing the error</a> occurs as the erroneous input is recieved
   </p>


2. The standard deviation values for GPS input and for subsequent process noise, is same. This is a bit confusing. Ideally it should be two separate values. 

3. As less calculations as possible withing loops. 
   This code is time critical. So should be designed to run in a very performant way. So as less calculations as possible. Some suggestions: 
   - Not too many particles (just enough to localize the car)
   - All one time calculations outside the loop.


4. Clash in mpping to landmarks. 
	Sometimes I saw, that few observations are mapped to same landmark. In that case, we remove the farther one, and retry. 
	(One option is also to just let it be, and let the weight calculation take care of it.)

5. In some iterations, weights are not updated, if their sum adds to zero. In that case, we keep a flag in resample also, and just return without resampling on the pre-existing weights. Apparently, that stabilizes it. 

Added on 1-August-2017 (Before second submit):
6. My project was not passed the first time. Even though I tried to explain my struggle and exploration in detail, with the simulator. Now with this superficial fix[1]. Without any code change in particle_filter.cpp. It works. I think simulator has a few issues. One which I have logged[2]. 

The other major review comment was regarding the method dataAssociation(). Actually its not over-engineered. But I am just taking care of multiple observations not getting marked to the same landmark. 

[1] https://discussions.udacity.com/t/beware-simulator-expects-yaw-in-0-to-2-pi-range-not-pi-to-pi/295952
[2] https://github.com/udacity/self-driving-car-sim/issues/47
