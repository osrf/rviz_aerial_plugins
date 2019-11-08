# RViz aerial plugins

These plugins allow to visualize data from the drone such as the battery status, the compass, the attitude display information or the flight information. Or even control the drone, for example, you can arm, disarm, takeoff, go to a 3D point (using geometry_msgs/PoseStamped) or land.

## Plugins
<center>

| Displays              | Panels         |
| --------------------- | -------------- |
| Battery               | Compass and ADI|
|                       | 3D Goal        |

</center>

## Screenshots

### Panels

You can add the panels using the upper menu: `Panels` -> `Add New Panel`:

<center>

  ![](img/panels.png)

</center>

You can choose in the `rviz_aerial_plugins` between:

#### Flight info: Compass, attitude display information

You can choose using the combobox which drone you want to display:

<center>

  ![](img/compass_and_adi.png)

</center>

#### Goal 3D

Using this panel you can visualize the current state of the drone. Clicking on the buttons you can arm, disarm, takeoff, land or move to a specific 3D goal.

Add an interactive marker to the display panel and select the topic `/drone_goal`. Then an interactive marker will appear in the 3D visualizer. You can move this marker and clicking in the *Go to*  button the drone will perform the movement.

<center>

  ![](img/goal3d.png)

</center>

<center>

  ![](img/drone_3dpose.png)

</center>

### Displays

From Display you can add Battery:

<center>

  ![](img/visualization.png)

</center>


#### Battery status

This display will shows the current voltage and the number of cells.

<center>

  ![](img/battery.png)

</center>
