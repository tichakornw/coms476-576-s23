## Lab 05 - Publisher and Subscriber

* Create a directory to store your python code

```
$ roscd cs476
$ mkdir scripts
$ cd scripts
```

* Add `talker.py` and `listener.py` to the new directory and make them executable

```
$ chmod +x talker.py
$ chmod +x listener.py
```

* Add the following to your CMakeLists.txt. This makes sure the python script gets installed properly, and uses the right python interpreter

```
catkin_install_python(PROGRAMS
  scripts/talker.py
  scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

* Change the listener() function of `listener.py` to

```
def listener():
    rospy.init_node("listener", anonymous=True)
    msg = rospy.wait_for_message("chatter", FloatArray)
    callback(msg)
```

* Build your nodes

```
$ cd ~/catkin_ws
$ catkin_make
```

* Run `roscore`, `talker.py`, and `listener.py` as described in Lab 2.