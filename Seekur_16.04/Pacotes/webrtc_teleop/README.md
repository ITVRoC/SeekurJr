# Iniciar comunicação utilizando webrtc

# Iniciar roscore
```sh
$ roscore
```

# Iniciar signaling do webrtc
```sh
$ rosrun ros_webrtc ros_webrtc_signaling _host:=0.0.0.0
```

# Iniciar o envio de video/audio/dados
```sh
$ roslaunch ros_webrtc ros_webrtc.launch
```


# Compilar unity plugin
```sh
 $ mkdir webrtc-checkout
 $ cd webrtc-checkout
 $ webrtc-checkout>fetch --nohooks webrtc
 $ webrtc-checkout>gclient sync
 $ webrtc-checkout>cd src
 $ webrtc-checkout\src>git checkout master
 $ webrtc-checkout\src>git new-branch your-branch-name
 $ webrtc-checkout\src>gn gen --ide=vs out/VS 
 $ webrtc-checkout\src>ninja -C out/VS/ webrtc_unity_plugin
 ```
 copiar a pasta unityplugin que está dentro de webrtc_teleop substituindo a pasta unityplugin  dentro de webrtc-checkout\src\out\VS
