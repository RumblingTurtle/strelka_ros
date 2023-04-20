import plotly.express as px
import numpy as np

with open('results.txt', 'r') as f:
    cost = [float(line.split(None, 1)[0]) * (-1) for line in f]

cost_2d = np.array(cost).reshape(4,7)
print(cost_2d)
max_velocity_x_range = np.arange(0.10,0.2501, 0.05)

max_curvature_range = np.arange(0.8,1.1, 0.05)
print(max_velocity_x_range)
print(cost_2d.shape,max_velocity_x_range.shape, max_curvature_range.shape)
fig = px.imshow(cost_2d,
                labels=dict(x="Maximum velocity along x", y="Curvature Threshold", color="Cost"),
                x=max_curvature_range,
                y=max_velocity_x_range, text_auto=True, aspect="auto")
fig.show()
