### Summary of Observations and learnings

1. Input data: 
   A big hurdle is the noise in the input data. One input yaw_rate value is 62.697. As soon as its recieved the yaw error goes out of range. 


2. The standard deviation values for GPS input and for subsequent process noise, is same. This is a bit confusing. Ideally it should be two separate values. 

3. As less calculations as possible withing loops. 
   This code is time critical. So should be designed to run in a very performant way. So as less calculations as possible. Some suggestions: 
   a. Not too many particles (just enough to localize the car)
   b. All one time calculations outside the loop.


4. Clash in mpping to landmarks. 
	Sometimes I saw, that few observations are mapped to same landmark. In that case, we remove the farther one, and retry. 
	(One option is also to just let it be, and let the weight calculation take care of it.)