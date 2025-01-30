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
  * To have all relevant files beeing placed in the correct subfolders by the code generator it is best practice to copy the carla_msgs folder in parallel to the other folders of your ROS2 system first and execute the generator from the respective ROS2 folder e.g.
    ```
    sudo cp -r carla_msgs /opt/ros/jazzy/share
    Fast-DDS-GEN/scripts/fastddsgen -d <home>/output-code -I /opt/ros/jazzy/share/ -typeros2 carla_msgs/msg/*.idl
    ```
    In case you get errors in some of the idl files: add "#pragma once" directive to those idls to ensure they are only included once by the generator.
    In some cases you will have to rename variables because of name clashes within different sub-namespaces which the fastddsgen generator is not able to distiguish. Easiest workaround for variables is placing a "_" in front of the name, so the output will be the same as expected. On class files append e.g. "BLABLA" and later perform a search and replace. Alternatively wait until the generator is fixed and works properly.

If the FastDDS Version is updated. The types in this folder have to be updated as above discribed. In addition, if one wants to keep the copyless operation on Image data, the header in fastcrd/Cdr.h has to be updated with the new version delivered by eProsima and the allocator extensions on the templates have to be applied there, too. Similarly, the (newly generated) sensor_msgs/msg/Image files have to be updated accordingly with ImageT<ALLOCATOR> type definitions for the same. In the end, quite some typing effort, otherwhise the data has to be copied into the standard allocated vector upfront passing to the deserialization of DDS (which again copies the data).

