---
title: Failed build job {{ env.UPSTREAM_TYPE }} ({{ env.DISTRO }}/{{ env.REPO }})
labels: CI
---
The scheduled build for branch `{{ env.REF }}` failed.

**Upstream build type:** {{ env.UPSTREAM_TYPE }}
**ROS DISTRO:** {{ env.DISTRO }}
**Repo:** {{ env.REPO }}

Please check the log output for details: {{ env.URL }}

Please note that this issue has different implications based on the build type. To get an idea of
the complete situation, please have a look at the [current build
status](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ci_status.md)

- If the *Upstream build type* is "binary":
  - If the *Repo* is "testing", it is well possible that an upstream package did make an
    API-breaking change that hasn't been released, yet. If the semi-binary builds are OK, this
    package has made the necessary changes, already. It is expected to sort itself out by itself,
    once the upstream package has been released. If the semi-binary builds are failing, as well,
    this package has to be updated.
  - If the *Repo* is "main", but the "testing" builds are green, we are only waiting for a Sync
    to happen.
- If the *Upstream build type* is "semi-binary":
  - If both, main and testing, are failing, there has been an API-breaking change in an upstream
    package. This package should get updated soon. If the upstream package gets released without
    updating this package, the "binary / testing" builds will also fail. In this case, this package
    needs to get updated and released ASAP.
  - If not both, main and testing, are failing, there is an upstream dependency with an API-breaking
    change that is not part of the upstream workspace. This should rarely happen and needs action
    from the package maintainers.
