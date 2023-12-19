## Issue tracking

If you have questions, suggestions or found a bug, feel free to open an [issue](https://github.com/kroshu/kuka_drivers/issues).
When filing an issue, please check open issues to make sure somebody else hasn't already reported it. Please try to include as much information as you can, including:
- A reproducible test case or series of steps
- The version/commit hash of our code being used
- Any modifications you've made relevant to the bug
- Anything unusual about your environment or deployment

Please do not add labels to the issue, that should be handled by the maintainers.

## Contributing

Contributions via pull requests are much appreciated. Before sending us a pull request, please ensure that:

- Your PR addresses only one issue
- Your PR has a descriptive title and a short summary
- All pipelines are green, including
    - Industrial CI
    - Sonarcloud
    - Spell checks
    - Linters

To send us a pull request, please:
- Discuss the proposed changes with the maintainers, preferably in an issue
- Fork the repository
- Modify the source focusing on the specific change you are contributing
- Ensure that local tests pass (`colcon test` and `pre-commit` run)
- Pay attention to any automated CI failures reported in the pull request, and stay involved in the conversation

## Licensing
Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

