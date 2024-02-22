import tkinter as tk
import numpy as np

WIDTH = 1000
HEIGHT = 1000

class DrawingApp:
    def __init__(self, root, width=400, height=400):
        self.root = root
        self.root.title("Drawing App")

        # Create a canvas
        self.canvas = tk.Canvas(root, width=width, height=height, bg="white")
        self.canvas.pack()

        # Initialize an empty list to store points
        self.points = []

        # Bind mouse events
        self.canvas.bind("<Button-1>", self.start_drawing)
        self.canvas.bind("<B1-Motion>", self.draw)

    def start_drawing(self, event):
        # Record the starting point
        self.points.append((event.x, event.y))

    def draw(self, event):
        # Draw a line segment from the last point to the current point
        x, y = event.x, event.y
        self.canvas.create_line(self.points[-1][0], self.points[-1][1], x, y, fill="black", width=5)

        # Update the point list
        self.points.append((x, y))

if __name__ == "__main__":
    root = tk.Tk()
    app = DrawingApp(root, width=WIDTH, height=HEIGHT)
    root.mainloop()
    # convert points to numpy array and csv
    points = np.array(app.points, dtype=float)
    points /= [WIDTH, HEIGHT]
    print(points)
    np.save("points.npy", points)
    # save as csv
    points = np.round(points, 3)
    # it is appearing with many digits and e-1, i don't want that
    # IT CREATES the header like # x,y, delete the # to avoid it
    np.savetxt("sketchbook/draw_csv_round/data/points.csv", points, delimiter=",", fmt="%.3f", header="x,y", comments="")
    print("Done")
    