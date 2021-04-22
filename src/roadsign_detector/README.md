# Roadsign detector

## RUN
Run the node (roscore should be running first)
```
rosrun roadsign_detector roadsign_detector
```

or you can use launch file (no roscore required)
```
roslaunch roadsign_detector roadsign_detector.launch
```

By default node reads images from ```/camera/image``` topic.

You can adjust algorithm parameters in the launch file.

## DEPENDENCIES
When using movie_launch to publish movies from *.mp4 file you shuld compile movie_publisher package from source changing output format to 'rgb8' in movie_publisher_node file. Also there are some issues with this package and additional dependency moviepy needs to be installed (see [https://github.com/peci1/movie_publisher](https://github.com/peci1/movie_publisher)).