# Custom message to send an image along with an integer value.

# Why do we need to define a custom message?

# This project requires computing the delay between when the image is produced and when it is received back by the Crazyflie.
# To accurately compute this delay, we need to use the same clock from the same device.
# The true delay is calculated using the Crazyflie's clock.
# The timestamp associated with the image is received from the img_streamer node along with the image.
# We use this custom message to pass both the image and the timestamp to the orb_slam3 node.
# The orb_slam3 node computes the position estimate and sends back the pose with the timestamp to the img_streamer node.
# The img_streamer node then sends both pieces of information to the Crazyflie.
# The drone can now compute the delay by comparing the timestamp when the image was created (passed with the pose) and the current time.

sensor_msgs/Image image
int32 integer
