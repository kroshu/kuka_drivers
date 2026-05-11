^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kuka_iiqka_eac_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix cycle time publisher namespace and parameter access rights (`#330 <https://github.com/kroshu/kuka_drivers/issues/330>`_) (`#331 <https://github.com/kroshu/kuka_drivers/issues/331>`_)
  * ci stabilization
  * format
  ---------
  (cherry picked from commit 85508cb0f08fcc87372f9d21d457e67f9c5a9ae8)
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
  Co-authored-by: Svastits <49677296+Svastits@users.noreply@github.com>
* Add option to lock memory (`#324 <https://github.com/kroshu/kuka_drivers/issues/324>`_) (`#325 <https://github.com/kroshu/kuka_drivers/issues/325>`_)
  * add mlockall
  * lock_memory
  * doc
  * format
  * bool fixes
  * fix
  * unlock memory
  * format
  (cherry picked from commit e9adb49e41fda2b86bdf686bc0dd18e273313339)
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
* Scheduling diagnostics (backport `#317 <https://github.com/kroshu/kuka_drivers/issues/317>`_) (`#318 <https://github.com/kroshu/kuka_drivers/issues/318>`_)
  * Scheduling diagnostics (`#317 <https://github.com/kroshu/kuka_drivers/issues/317>`_)
  * copyrights
  * timing diagnostics
  (cherry picked from commit 6fa7037be2d1f448c95bd246ae000b41bd6442b4)
  # Conflicts:
  #	kuka_rsi_driver/src/hardware_interface_rsi_base.cpp
  * conflict
  ---------
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
* Fixed the paths used for namespace organization (`#311 <https://github.com/kroshu/kuka_drivers/issues/311>`_) (`#315 <https://github.com/kroshu/kuka_drivers/issues/315>`_)
  * EAC driver namespace fix
  * RSI namespace fix
  * Sunrise namespace fix
  * format fixes
  * Increased test time for more reliable results
  (cherry picked from commit 517e495eedd157752613dded19528e512192c53e)
  Co-authored-by: Karnitscher <karnitschervilmos@edu.bme.hu>
* cleanup (`#307 <https://github.com/kroshu/kuka_drivers/issues/307>`_) (`#308 <https://github.com/kroshu/kuka_drivers/issues/308>`_)
  (cherry picked from commit e5718bbb7196ed59c78a2ffd2319dbe40f441d39)
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
* Backport comment fixes (`#297 <https://github.com/kroshu/kuka_drivers/issues/297>`_) (`#298 <https://github.com/kroshu/kuka_drivers/issues/298>`_)
  * fixes
  * topic name
  ---------
  (cherry picked from commit da56b32c585536e62f26936bbc2ea0de4e61344c)
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
  Co-authored-by: Sándor Komáromi <sanyi.komaromi@gmail.com>
* Add `commanded_position` interface to hw_ifs and `joint_group_impedance_controller` (backport `#295 <https://github.com/kroshu/kuka_drivers/issues/295>`_) (`#296 <https://github.com/kroshu/kuka_drivers/issues/296>`_)
  * Add `commanded_position` interface to hw_ifs and `joint_group_impedance_controller` (`#295 <https://github.com/kroshu/kuka_drivers/issues/295>`_)
  * add new state interface
  * part1
  * publish values
  * fixes
  * sunrise commanded_pos
  * format
  * doc update
  ---------
  Co-authored-by: Svastits <49677296+Svastits@users.noreply@github.com>
  (cherry picked from commit ca21bb671c648ee746d6d364d68b7213b81b9a25)
  * backport fix
  ---------
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
  Co-authored-by: Svastits <49677296+Svastits@users.noreply@github.com>
  Co-authored-by: Sándor Komáromi <sanyi.komaromi@gmail.com>
* Make CPU affinities and realtime thread priority configurable for RSI driver (backport `#291 <https://github.com/kroshu/kuka_drivers/issues/291>`_) (`#292 <https://github.com/kroshu/kuka_drivers/issues/292>`_)
  * Make CPU affinities and realtime thread priority configurable for RSI driver (`#291 <https://github.com/kroshu/kuka_drivers/issues/291>`_)
  * non_rt_cores
  * affinity for rt
  * launch extension
  * format
  * int
  * default value
  * doc
  * copy
  * extend other launch files
  * doc
  * add missing
  ---------
  Co-authored-by: Svastits <49677296+Svastits@users.noreply@github.com>
  Co-authored-by: Svastits <svastits1@gmail.com>
  (cherry picked from commit 3e7e3cf5713ee527a9a0769458821a212014c730)
  # Conflicts:
  #	doc/wiki/1_iiQKA_EAC.md
  #	doc/wiki/2_KSS_RSI.md
  #	doc/wiki/3_iiQKA.OS2_RSI.md
  #	doc/wiki/4_Sunrise_FRI.md
  #	kuka_iiqka_eac_driver/launch/startup.launch.py
  #	kuka_rsi_driver/launch/startup.launch.py
  #	kuka_sunrise_fri_driver/launch/startup.launch.py
  * merge conflicts
  * doc conflicts
  ---------
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
* fix topic name (`#204 <https://github.com/kroshu/kuka_drivers/issues/204>`_) (`#290 <https://github.com/kroshu/kuka_drivers/issues/290>`_)
  (cherry picked from commit 6eae82848bfc31258ec8732e555de6966272bfe1)
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
  Co-authored-by: Svastits <svastits1@gmail.com>
* Update EAC driver launch file with new driver_version argument (`#288 <https://github.com/kroshu/kuka_drivers/issues/288>`_) (`#289 <https://github.com/kroshu/kuka_drivers/issues/289>`_)
  * add driver version too eac launch file
  * Update startup.launch.py
  ---------
  (cherry picked from commit 05f9ae8dcb5419874f236877be9cc787b3279c0d)
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
  Co-authored-by: Svastits <svastits1@gmail.com>
* Add mxAutomation support and refactor (backport `#284 <https://github.com/kroshu/kuka_drivers/issues/284>`_) (`#287 <https://github.com/kroshu/kuka_drivers/issues/287>`_)
  * Add mxAutomation support and refactor (`#284 <https://github.com/kroshu/kuka_drivers/issues/284>`_)
  * copies
  * fixes
  * make it work
  * do not support model verification for mxa 3
  * test
  * format
  * format
  * namespace alignments
  * refactor 1. steps
  * CreateRobotInstance
  * more unification
  * read-write
  * on_configure
  * eki init
  * export
  * rm non-base
  * extension methods
  * robot manager cleanup
  * format
  * export_state_interfaces
  * hwif cleanup
  * single robot ptr
  * use client port
  * event handler register change
  * fix
  * virtual fix
  * deactivate if motion not possible
  * cycle time
  * check dynamic cast
  * virtual destructor
  * extended
  * event observers
  * copyright
  * cr
  * Revert "copyright"
  This reverts commit 218ddf3378e167b381d5e5b142e0f5e07dcf7e25.
  * copyright
  * Update kuka_rsi_driver/launch/startup.launch.py
  Co-authored-by: Kristof Pasztor <100444459+pasztork@users.noreply.github.com>
  * Update kuka_rsi_driver/src/hardware_interface_mxa_rsi.cpp
  Co-authored-by: Kristof Pasztor <100444459+pasztork@users.noreply.github.com>
  * rm duplicate on_init
  * format
  * logger + dead code
  * write refactor
  * simplify cleanup+ config
  * sonar
  * format
  * sonar + timeouts
  ---------
  Co-authored-by: Svastits <49677296+Svastits@users.noreply@github.com>
  Co-authored-by: Kristof Pasztor <100444459+pasztork@users.noreply.github.com>
  (cherry picked from commit acb5556a9327b7cfd20932c9479ddf1ea66eae3d)
  # Conflicts:
  #	controllers/kuka_kss_message_handler/src/kuka_kss_message_handler.cpp
  #	kuka_rsi_driver/include/kuka_rsi_driver/hardware_interface_eki_rsi.hpp
  #	kuka_rsi_driver/include/kuka_rsi_driver/hardware_interface_rsi_only.hpp
  #	kuka_rsi_driver/launch/startup.launch.py
  #	kuka_rsi_driver/src/hardware_interface_eki_rsi.cpp
  #	kuka_rsi_driver/src/hardware_interface_rsi_only.cpp
  #	kuka_rsi_driver/src/robot_manager_node_rsi_only.cpp
  * mrege conflicts
  * build fixes
  * cleanup drive state command if
  * only call write in active
  * format
  * test fix
  ---------
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
  Co-authored-by: Svastits <svastits1@gmail.com>
* Contributors: mergify[bot]

1.0.0 (2025-10-03)
------------------
* Add Gazebo support
* Smaller bugfixes
* Contributors: Komáromi Sándor, Kristof Pasztor, Áron Svastits, Levente Nas, Kristófi Mihály

0.9.2 (2024-07-10)
------------------
* Fix GCC warning causing unstable build

0.9.1 (2024-07-08)
------------------
* Add missing test dependency

0.9.0 (2024-07-08)
------------------
* Add package with driver for KUKA iiQKA robots
* Contributors: Aron Svastits, Gergely Kovacs, Mark Szitanics, Sandor Komaromi
