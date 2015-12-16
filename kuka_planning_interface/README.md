# Kuka planning interface

launch server
```
$ roslaunch pour_kuka kuka_server.launch
```

launch client
```
$ roslaunch pour_kuka kuka_client.launch
```

From command line, type desired action:
```
$ rosservice call /control_cmd_interface/kuka_cmd 'home'
$ rosservice call /control_cmd_interface/kuka_cmd 'pour'
$ rosservice call /control_cmd_interface/kuka_cmd 'back'
```
