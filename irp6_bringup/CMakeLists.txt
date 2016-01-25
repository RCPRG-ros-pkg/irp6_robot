cmake_minimum_required(VERSION 2.8.3)
project(irp6_bringup)
find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS xacro)

foreach(dir config launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_install_python(PROGRAMS
scripts/conveyor.py
scripts/irp6ot_tfg.py
scripts/irp6otm_force_control.py
scripts/irp6otm_multi_trajectory.py
scripts/irp6otm_synchro.py
scripts/irp6p_tfg.py
scripts/irp6pm_circle.py
scripts/irp6pm_force_control.py
scripts/irp6pm_front_position.py
scripts/irp6pm_multi_trajectory.py
scripts/irp6pm_simple_trajectory.py
scripts/irp6pm_synchro.py
scripts/update_monitor.py
scripts/IRPOS/irpos_test.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})