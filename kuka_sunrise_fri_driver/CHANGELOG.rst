^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kuka_sunrise_fri_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added synchronus cycle time change to sunrise similar to RSI pattern (`#340 <https://github.com/kroshu/kuka_drivers/issues/340>`_) (`#341 <https://github.com/kroshu/kuka_drivers/issues/341>`_)
  * Addied synchronus cycle time change to sunrise similar to RSI pattern
  * format fixes
  * Reduced variables
  * Review changes
  ---------
  (cherry picked from commit 27c844aeb6d024f2ad79753394501ac41192267a)
  Co-authored-by: Karnitscher <karnitschervilmos@edu.bme.hu>
  Co-authored-by: Copilot <copilot@github.com>
* Synchronize update rate of controller manager (backport `#323 <https://github.com/kroshu/kuka_drivers/issues/323>`_) (`#332 <https://github.com/kroshu/kuka_drivers/issues/332>`_)
  * Synchronize update rate of controller manager (`#323 <https://github.com/kroshu/kuka_drivers/issues/323>`_)
  * Changed sunrise param naming outward name cycle_time server side send_period_ms
  * Added cycle time parameter to rsi friver
  * made RSI cycle time on the fly modifiable
  * RSI/EKI minimized code duplication during control mode switch
  * Format fixes and sentry fixes
  * EKI/RSI update rate same as cycle time
  * Added update rate fix to fri
  * Format fixes
  * Review changes
  * Added extra publishing to configure function
  * Format fixes
  * Improved logging
  * format fixes
  * Update kuka_rsi_driver/include/kuka_rsi_driver/robot_manager_base.hpp
  Added class to CycleTime enum definition
  Co-authored-by: Kristof Pasztor <100444459+pasztork@users.noreply.github.com>
  * Wiki update
  * Wiki update update
  * Update rate fix after merge from cycle time branch
  * Control node calculates based on time passed independetly from update rate
  * Upgrade update rate into sync request
  * Reentrant fix
  * Added sync request
  * format fixes
  * Sentry fixes
  * Separeted cycletime validation and changing for stability
  * format fixes
  * format and sentry fixes
  * time and format fix
  * Sonar and sentry fixes
  ---------
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
  Co-authored-by: Kristof Pasztor <100444459+pasztork@users.noreply.github.com>
  (cherry picked from commit 71241b096666124f1dbb194f11dcc8cb95cf4c72)
  * clock fix
  * fixes
  ---------
  Co-authored-by: Karnitscher <karnitschervilmos@edu.bme.hu>
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
  Co-authored-by: Kristof Pasztor <100444459+pasztork@users.noreply.github.com>
* Fix cycle time publisher namespace and parameter access rights (`#330 <https://github.com/kroshu/kuka_drivers/issues/330>`_) (`#331 <https://github.com/kroshu/kuka_drivers/issues/331>`_)
  * ci stabilization
  * format
  ---------
  (cherry picked from commit 85508cb0f08fcc87372f9d21d457e67f9c5a9ae8)
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
  Co-authored-by: Svastits <49677296+Svastits@users.noreply@github.com>
* Add `cycle_time` parameters to drivers using RSI (`#322 <https://github.com/kroshu/kuka_drivers/issues/322>`_) (`#327 <https://github.com/kroshu/kuka_drivers/issues/327>`_)
  * Changed sunrise param naming outward name cycle_time server side send_period_ms
  * Added cycle time parameter to rsi friver
  * made RSI cycle time on the fly modifiable
  * RSI/EKI minimized code duplication during control mode switch
  * Format fixes and sentry fixes
  * Review changes
  * Added extra publishing to configure function
  * Format fixes
  * Improved logging
  * format fixes
  * Update kuka_rsi_driver/include/kuka_rsi_driver/robot_manager_base.hpp
  Added class to CycleTime enum definition
  * Wiki update
  * Wiki update update
  ---------
  (cherry picked from commit c33d47a2d9ef4ea522a58bf5f97d212cc031334d)
  Co-authored-by: Karnitscher <karnitschervilmos@edu.bme.hu>
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
  Co-authored-by: Kristof Pasztor <100444459+pasztork@users.noreply.github.com>
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
* Bugfixes
  * Fix sunrise activation
  * Fix impedance config publisher
  * Make server port configurable
* Update version and maintainers
* Contributors: Komáromi Sándor, Kristof Pasztor, Áron Svastits, Levente Nas, Kristófi Mihály

0.9.2 (2024-07-10)
------------------
* Fix GCC warning causing unstable build

0.9.1 (2024-07-08)
------------------
* Add missing test dependency

0.9.0 (2024-07-08)
------------------
* Add package with driver for KUKA Sunrise robots
* Contributors: Aron Svastits, Zoltan Resi, Gergely Kovacs
