To update the types within this folder one has to:
  
  * Checkout the github.com/carla-simulator/ros-carla-msgs repository
    ```git clone https://github.com/carla-simulator/ros-carla-msgs```
  * install ROS2 on the system and all message dependencies of the carla_msgs (see ros-carla-msgs docu)
  * in case the carla msg files are changed:
    - build the ROS2 package of the carla_msgs
    - copy the idl files from the build folder into the respective carla_msgs folder
    - revert the removal of "#pragma once" line within the overridden idls
    - add "#pragma once" directive to newly created idls
  * Install fastddsgen in a compatible version see https://fast-dds.docs.eprosima.com/en/latest/notes/versions.html#eprosima-products-compatibilityInstall
    Alternatively build the matching version on your own using gradle.
    ```
    git clone https://github.com/eProsima/Fast-DDS-Gen/
    cd Fast-DDS-GEN
    ./gradlew assemble
    ```
    If you are behind a proxy, don't forget to setup your ~/.gradle/gradle.properties file with the following:
    ```
    systemProp.https.proxyHost=proxy.company.net
    systemProp.https.proxyPort=8181
    systemProp.http.proxyHost=proxy.company.net
    systemProp.http.proxyPort=8181
    systemProp.https.nonProxyHosts=*.company.com|localhost
    ```
  * Call fastddsgen outside the carla_msgs folder e.g.
    ```fastddsgen -d code -I . -I /opt/ros/humble/share/ -typeros2 carla_msgs/msg/*.idl```
    In case you get errors in standard idl files: add "#pragma once" directive those idls
  * Then copy all required code into the respecitve subfolders in here and put rework include directives where necessary 
    as the fastddsgen generator is unfortunately not supporting to replicate the correct subdirectory structure in all cases yet.


If the FastDDS Version is updated. The types in this folder have to be updated as above discribed. In addition, if one wants to keep the copyless operation on Image data, the header in fastcrd/Cdr.h has to be updated with the new version delivered by eProsima and the allocator extensions on the templates have to be applied there, too. Similarly, the (newly generated) sensor_msgs/msg/Image files have to be updated accordingly with ImageT<ALLOCATOR> type definitions for the same. In the end, quite some typing effort, otherwhise the data has to be copied into the standard allocated vector upfront passing to the deserialization of DDS (which again copies the data).

