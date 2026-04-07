#!/usr/bin/env python3
# Copyright 2026, Universal Robots A/S
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Verify that all controllers defined in ur_controllers.yaml are documented.

Every controller listed in the controller_manager configuration must be mentioned
in at least one of the control mode documentation pages:
  - ur_robot_driver/doc/position_velocity_control.rst
  - ur_robot_driver/doc/force_torque_control.rst
  - ur_robot_driver/doc/utility_controllers.rst
"""

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent

CONTROLLERS_YAML = REPO_ROOT / "ur_robot_driver" / "config" / "ur_controllers.yaml"

DOC_PAGES = [
    REPO_ROOT / "ur_robot_driver" / "doc" / "position_velocity_control.rst",
    REPO_ROOT / "ur_robot_driver" / "doc" / "force_torque_control.rst",
    REPO_ROOT / "ur_robot_driver" / "doc" / "utility_controllers.rst",
]


def extract_controller_names(yaml_path: Path) -> list[str]:
    """Extract controller names from the controller_manager section of the YAML config.

    Uses simple regex parsing to avoid requiring the yaml package (which would
    fail on $(var ...) substitutions anyway).
    """
    controllers = []
    in_controller_manager = False
    indent_pattern = re.compile(r"^    (\w+):$")

    for line in yaml_path.read_text().splitlines():
        if line.startswith("controller_manager:"):
            in_controller_manager = True
            continue
        if in_controller_manager:
            if line and not line.startswith(" "):
                break
            match = indent_pattern.match(line)
            if match:
                name = match.group(1)
                if name != "ros__parameters":
                    controllers.append(name)

    return controllers


def main() -> int:
    if not CONTROLLERS_YAML.exists():
        print(f"ERROR: {CONTROLLERS_YAML} not found")
        return 1

    controllers = extract_controller_names(CONTROLLERS_YAML)
    if not controllers:
        print(f"ERROR: No controllers found in {CONTROLLERS_YAML}")
        return 1

    doc_content = ""
    missing_pages = []
    for page in DOC_PAGES:
        if not page.exists():
            missing_pages.append(str(page))
            continue
        doc_content += page.read_text()

    if missing_pages:
        print(f"ERROR: Documentation pages not found: {', '.join(missing_pages)}")
        return 1

    undocumented = []
    for controller in controllers:
        if controller not in doc_content:
            undocumented.append(controller)

    if undocumented:
        print("ERROR: The following controllers are not documented in any control mode page:")
        for name in undocumented:
            print(f"  - {name}")
        print()
        print("Each controller in ur_controllers.yaml must be mentioned in at least one of:")
        for page in DOC_PAGES:
            print(f"  - {page.relative_to(REPO_ROOT)}")
        return 1

    print(f"OK: All {len(controllers)} controllers are documented.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
