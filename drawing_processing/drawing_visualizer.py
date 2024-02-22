import numpy as np
import cv2
import argparse

# Parse the arguments
parser = argparse.ArgumentParser()
# if user passss --timelapse or -t, it will be stored in the variable timelapse as true, and a --csv argument
parser.add_argument("--timelapse", "-t", action="store_true", help="Show the drawing process")
parser.add_argument("--csv","-c", action="store_true", help="Load the points from a CSV file")
args = parser.parse_args()

# Load the points and the width and height
if not args.csv:
    points = np.load("points.npy")
    #print(points)
else:
    points = np.genfromtxt("points.csv", delimiter=",")
    #print(points)
WIDTH, HEIGHT = 500, 500

# Create a black image
img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)

# Draw the lines
for i in range(len(points) - 1):
    #points are normalized, so we need to multiply by the width and height
    x1, y1 = points[i] * [WIDTH, HEIGHT]
    x2, y2 = points[i + 1] * [WIDTH, HEIGHT]
    cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 255), 5)
    if args.timelapse:
        cv2.imshow("Image", img)
        cv2.waitKey(20)

# Show the image
cv2.imshow("Image", img)
cv2.waitKey(0)
cv2.destroyAllWindows()