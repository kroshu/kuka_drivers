^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kuka_drivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* cleanup (`#307 <https://github.com/kroshu/kuka_drivers/issues/307>`_) (`#308 <https://github.com/kroshu/kuka_drivers/issues/308>`_)
  (cherry picked from commit e5718bbb7196ed59c78a2ffd2319dbe40f441d39)
  Co-authored-by: Áron Svastits <49677296+Svastits@users.noreply.github.com>
* Add external axis support (backport `#305 <https://github.com/kroshu/kuka_drivers/issues/305>`_) (`#306 <https://github.com/kroshu/kuka_drivers/issues/306>`_)
  * Add external axis support (`#305 <https://github.com/kroshu/kuka_drivers/issues/305>`_)
  * Add external axis example
  * Fix pre-commit
  * Fix logger
  * Update RSI simulator for external axis support
  * Add joint config to RSI driver
  * Log joint configuration details in ConfigureJoints method
  * Align RSI simulator
  * Add external axes configuration documentation and examples
  * Fix typo
  * Fix link
  * Run pre-commit
  * Add external axes configuration documentation for KSS and iiQKA drivers
  * Move examples into separate repository
  * Make ConfigureJoints method const in hardware_interface_rsi_base
  * Format ConfigureJoints method declaration for pre-commit
  (cherry picked from commit 3d77a4178a0fb77a1d00ac0272c2754d721a5d78)
  # Conflicts:
  #	controllers/kuka_kss_message_handler/package.xml
  #	doc/wiki/Home.md
  #	examples/iiqka_moveit_example/CHANGELOG.rst
  #	examples/iiqka_moveit_example/include/iiqka_moveit_example/moveit_example.hpp
  #	examples/iiqka_moveit_example/package.xml
  * Solve merge conflicts
  ---------
  Co-authored-by: Kristof Pasztor <100444459+pasztork@users.noreply.github.com>
  Co-authored-by: Kristof Pasztor <pasztor.kristof.kp@gmail.com>
* Contributors: mergify[bot]

1.0.0 (2025-10-03)
------------------
* Redesign RSI communication to make it real-time safe
* Add EKI wrapper for starting RSI
* Add IO support
* Add iiQKA.OS2 support
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
* Add meta-package for KUKA drivers
* Contributors: Aron Svastits
