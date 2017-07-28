### Summary of Observations and learnings

1. Input data: 
   A big hurdle is the noise in the input data. One input yaw_rate value is 62.697. As soon as its recieved the yaw error goes out of range. 
   I suspect this could be simulator bug. I am using the Ubuntu version v1.45)


2. The standard deviation values for GPS input and for subsequent process noise, is same. This is a bit confusing. Ideally it should be two separate values. 

3. As less calculations as possible withing loops. 
   This code is time critical. So should be designed to run in a very performant way. So as less calculations as possible. Some suggestions: 
   - Not too many particles (just enough to localize the car)
   - All one time calculations outside the loop.


4. Clash in mpping to landmarks. 
	Sometimes I saw, that few observations are mapped to same landmark. In that case, we remove the farther one, and retry. 
	(One option is also to just let it be, and let the weight calculation take care of it.)

5. In some iterations, weights are not updated, if their sum adds to zero. In that case, we keep a flag in resample also, and just return without resampling on the pre-existing weights. Apparently, that stabilizes it. 