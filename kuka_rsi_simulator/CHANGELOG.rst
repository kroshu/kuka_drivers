^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kuka_rsi_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2026-05-11)
------------------
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
* Update version and maintainers
* Contributors: Komáromi Sándor

0.9.2 (2024-07-10)
------------------
* Fix GCC warning causing unstable build

0.9.1 (2024-07-08)
------------------
* Add missing test dependency

0.9.0 (2024-07-08)
------------------
* Add package with simulator for KUKA RSI driven robots
* Contributors: Aron Svastits, Lars Tingelstad
