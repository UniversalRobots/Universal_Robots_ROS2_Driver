#!/usr/bin/env python
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

"""
Unit tests for the ``tf_prefix`` support in the MoveIt SRDF.

These tests expand the SRDF (and, for the consistency check, the driver URDF)
with xacro and inspect the resulting names. They guard against a regression of
the fix for the case where the SRDF hardcoded unprefixed link/joint names while
the URDF supports a ``tf_prefix`` (see the "Joint prefix supported in URDF, but
not in MoveIt SRDF" issue): with a non-empty prefix MoveIt could not build the
``ur_manipulator`` group because the SRDF referenced joints/links that did not
exist in the URDF.

The tests only perform static xacro expansion, so they do not require URSim or a
running MoveIt instance and run as part of the regular (non-integration) test
set.
"""

import os
import xml.etree.ElementTree as ET

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory


def _expand_xacro(path, mappings):
    """Expand a xacro file with the given mappings and return the XML root."""
    doc = xacro.process_file(path, mappings=mappings)
    return ET.fromstring(doc.toxml())


def _srdf_path():
    return os.path.join(get_package_share_directory("ur_moveit_config"), "srdf", "ur.srdf.xacro")


def _urdf_path():
    return os.path.join(get_package_share_directory("ur_robot_driver"), "urdf", "ur.urdf.xacro")


def _srdf_referenced_names(root):
    """Collect every link/joint name referenced by the SRDF.

    This covers the chain endpoints, the joints listed in the group states and
    the link pairs in the ``disable_collisions`` entries -- i.e. all names that
    must exist in the URDF for MoveIt to accept the SRDF.
    """
    names = set()
    for chain in root.iter("chain"):
        names.add(chain.get("base_link"))
        names.add(chain.get("tip_link"))
    for group_state in root.iter("group_state"):
        for joint in group_state.iter("joint"):
            names.add(joint.get("name"))
    for disable in root.iter("disable_collisions"):
        names.add(disable.get("link1"))
        names.add(disable.get("link2"))
    names.discard(None)
    return names


def _urdf_names(root):
    """Collect every link and joint name defined in the URDF."""
    names = set()
    for link in root.iter("link"):
        names.add(link.get("name"))
    for joint in root.iter("joint"):
        names.add(joint.get("name"))
    names.discard(None)
    return names


@pytest.mark.parametrize("tf_prefix", ["", "test_"])
def test_srdf_prefixing(tf_prefix):
    """The SRDF must apply ``tf_prefix`` to every referenced link and joint."""
    root = _expand_xacro(_srdf_path(), {"name": "ur", "tf_prefix": tf_prefix})
    referenced = _srdf_referenced_names(root)

    assert referenced, "No link/joint references found in the expanded SRDF"

    if tf_prefix:
        unprefixed = sorted(n for n in referenced if not n.startswith(tf_prefix))
        assert not unprefixed, f"SRDF names missing the '{tf_prefix}' prefix: {unprefixed}"
    else:
        # Regression guard: with an empty prefix no name may accidentally carry a
        # prefix, and the previously broken behaviour (dropped prefixing) must
        # keep producing the plain names.
        prefixed = sorted(n for n in referenced if n.startswith("test_"))
        assert not prefixed, f"Unexpected prefixed SRDF names with empty prefix: {prefixed}"

    # The group name is intentionally kept unprefixed so that the shipped
    # configs (kinematics.yaml, ur_servo.yaml, moveit.rviz) keep working.
    group_names = {group.get("name") for group in root.iter("group")}
    assert "ur_manipulator" in group_names


@pytest.mark.parametrize("tf_prefix", ["", "test_"])
def test_srdf_matches_urdf(tf_prefix):
    """Every link/joint referenced by the SRDF must exist in the URDF.

    This reproduces the exact failure from the issue ("joints/links referenced
    in the SRDF are not known to the URDF") without launching MoveIt.
    """
    srdf_root = _expand_xacro(_srdf_path(), {"name": "ur", "tf_prefix": tf_prefix})
    urdf_root = _expand_xacro(
        _urdf_path(), {"name": "ur", "ur_type": "ur5e", "tf_prefix": tf_prefix}
    )

    referenced = _srdf_referenced_names(srdf_root)
    urdf_names = _urdf_names(urdf_root)

    missing = sorted(referenced - urdf_names)
    assert not missing, (
        f"SRDF references names unknown to the URDF for tf_prefix='{tf_prefix}': {missing}"
    )
