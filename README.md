# Vehicle Recognition
Vehicle recognition from video.

With this application, you can determine the types of vehicles in the video which you processing on Matlab.
This application uses Kalman filter.

<b>What is the Kalman Filter ?</b>

The algorithm works in a two-step process. In the prediction step, the Kalman filter produces estimates of the current state variables, along with their uncertainties. Once the outcome of the next measurement (necessarily corrupted with some amount of error, including random noise) is observed, these estimates are updated using a weighted average, with more weight being given to estimates with higher certainty. The algorithm is recursive. It can run in real time, using only the present input measurements and the previously calculated state and its uncertainty matrix; no additional past information is required.

<b>How to Use ?</b>

<code>video = mmreader(<b>'15sn.avi'</b>);</code>

Record the video and change the bold text with your video's name and run it on Matlab !

<b>Notes</b>

This works with only .avi videos.

