import numpy as np
import cv2


lala =np.load("dcam.npy")
ya = lala.reshape(-1, 2048)

# Display the image with padding using OpenCV
cv2.imshow("Image with Padding", ya)
cv2.waitKey(0)  # Wait until a key is pressed to close the image
cv2.destroyAllWindows()  # Close the image window

