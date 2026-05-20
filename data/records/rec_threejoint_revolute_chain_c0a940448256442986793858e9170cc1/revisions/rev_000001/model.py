from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PLATE_THICKNESS = 0.010
PLATE_WIDTH = 0.080
PLATE_HEIGHT = 0.095
MOUNT_HOLE_RADIUS = 0.0045
MOUNT_HOLE_Y = 0.025
MOUNT_HOLE_Z = 0.030

BARREL_RADIUS = 0.0055
CENTER_BARREL_LENGTH = 0.006
OUTER_BARREL_LENGTH = 0.006
OUTER_BARREL_Y = 0.006
LINK_THICKNESS = CENTER_BARREL_LENGTH
LINK_WIDTH = 0.020

ROOT_AXIS_X = 0.028
LINK1_LENGTH = 0.102
LINK2_LENGTH = 0.086
END_TAB_LENGTH = 0.042


def _add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_y_barrel(
    part,
    *,
    name: str,
    x: float,
    y: float,
    z: float,
    radius: float,
    length: float,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=part.meta["material"],
        name=name,
    )


def _populate_mounting_plate(part) -> None:
    material = part.meta["material"]
    _add_box(
        part,
        name="plate_slab",
        size=(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT),
        center=(0.0, 0.0, 0.0),
        material=material,
    )
    ear_length = ROOT_AXIS_X - PLATE_THICKNESS / 2.0
    ear_center_x = (ROOT_AXIS_X + PLATE_THICKNESS / 2.0) / 2.0
    for side_name, ear_y in (("lower", -OUTER_BARREL_Y), ("upper", OUTER_BARREL_Y)):
        _add_box(
            part,
            name=f"{side_name}_ear_block",
            size=(ear_length, OUTER_BARREL_LENGTH, 0.018),
            center=(ear_center_x, ear_y, 0.0),
            material=material,
        )
        _add_y_barrel(
            part,
            name=f"{side_name}_ear_barrel",
            x=ROOT_AXIS_X,
            y=ear_y,
            z=0.0,
            radius=BARREL_RADIUS,
            length=OUTER_BARREL_LENGTH,
        )


def _populate_link(part, *, length: float, narrow: bool = False) -> None:
    material = part.meta["material"]
    spine_start = 0.014
    spine_end = length - 0.030
    spine_width = 0.018 if narrow else LINK_WIDTH
    _add_y_barrel(
        part,
        name="root_barrel",
        x=0.0,
        y=0.0,
        z=0.0,
        radius=BARREL_RADIUS,
        length=CENTER_BARREL_LENGTH,
    )
    _add_box(
        part,
        name="root_neck",
        size=(0.014, LINK_THICKNESS, 0.016),
        center=(0.007, 0.0, 0.0),
        material=material,
    )
    _add_box(
        part,
        name="spine",
        size=(spine_end - spine_start, LINK_THICKNESS, spine_width),
        center=((spine_start + spine_end) / 2.0, 0.0, 0.0),
        material=material,
    )
    _add_box(
        part,
        name="fork_bridge",
        size=(0.018, 0.018, 0.012),
        center=(length - 0.021, 0.0, 0.0),
        material=material,
    )
    for side_name, arm_y in (("lower", -OUTER_BARREL_Y), ("upper", OUTER_BARREL_Y)):
        _add_box(
            part,
            name=f"{side_name}_fork_arm",
            size=(0.018, OUTER_BARREL_LENGTH, 0.014),
            center=(length - 0.009, arm_y, 0.0),
            material=material,
        )
        _add_y_barrel(
            part,
            name=f"{side_name}_fork_barrel",
            x=length,
            y=arm_y,
            z=0.0,
            radius=BARREL_RADIUS,
            length=OUTER_BARREL_LENGTH,
        )


def _populate_end_tab(part) -> None:
    material = part.meta["material"]
    _add_y_barrel(
        part,
        name="root_barrel",
        x=0.0,
        y=0.0,
        z=0.0,
        radius=BARREL_RADIUS,
        length=CENTER_BARREL_LENGTH,
    )
    _add_box(
        part,
        name="root_neck",
        size=(0.012, LINK_THICKNESS, 0.014),
        center=(0.006, 0.0, 0.0),
        material=material,
    )
    _add_box(
        part,
        name="tab_body",
        size=(END_TAB_LENGTH - 0.010, LINK_THICKNESS, 0.018),
        center=((END_TAB_LENGTH + 0.010) / 2.0, 0.0, 0.0),
        material=material,
    )
    _add_box(
        part,
        name="tab_nose",
        size=(0.010, LINK_THICKNESS, 0.014),
        center=(END_TAB_LENGTH - 0.005, 0.0, 0.0),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_revolute_chain")

    steel_dark = model.material("steel_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.42, 0.45, 0.49, 1.0))
    steel_light = model.material("steel_light", rgba=(0.60, 0.63, 0.67, 1.0))

    mounting_plate = model.part("mounting_plate", meta={"material": steel_dark})
    _populate_mounting_plate(mounting_plate)

    link_1 = model.part("link_1", meta={"material": steel_mid})
    _populate_link(link_1, length=LINK1_LENGTH)

    link_2 = model.part("link_2", meta={"material": steel_light})
    _populate_link(link_2, length=LINK2_LENGTH, narrow=True)

    end_tab = model.part("end_tab", meta={"material": steel_dark})
    _populate_end_tab(end_tab)

    common_limits = MotionLimits(effort=12.0, velocity=2.2, lower=0.0, upper=1.55)

    model.articulation(
        "plate_to_link_1",
        ArticulationType.REVOLUTE,
        parent=mounting_plate,
        child=link_1,
        origin=Origin(xyz=(ROOT_AXIS_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_2_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_tab,
        origin=Origin(xyz=(LINK2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mounting_plate = object_model.get_part("mounting_plate")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_tab = object_model.get_part("end_tab")
    root_joint = object_model.get_articulation("plate_to_link_1")
    mid_joint = object_model.get_articulation("link_1_to_link_2")
    distal_joint = object_model.get_articulation("link_2_to_end_tab")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for joint_obj in (root_joint, mid_joint, distal_joint):
        ctx.check(
            f"{joint_obj.name}_bends_in_one_plane",
            tuple(round(v, 6) for v in joint_obj.axis) == (0.0, -1.0, 0.0),
            details=f"expected axis (0, -1, 0), got {joint_obj.axis}",
        )

    ctx.expect_contact(mounting_plate, link_1, name="root_joint_has_physical_contact")
    ctx.expect_contact(link_1, link_2, name="middle_joint_has_physical_contact")
    ctx.expect_contact(link_2, end_tab, name="distal_joint_has_physical_contact")

    with ctx.pose({root_joint: 0.65, mid_joint: 0.75, distal_joint: 0.70}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_pose")
        end_pos = ctx.part_world_position(end_tab)
        plate_pos = ctx.part_world_position(mounting_plate)
        ok = (
            end_pos is not None
            and plate_pos is not None
            and end_pos[0] > ROOT_AXIS_X + 0.05
            and end_pos[2] > plate_pos[2] + 0.08
        )
        ctx.check(
            "folded_chain_lifts_distal_tab_upward",
            ok,
            details=f"mounting_plate={plate_pos}, end_tab={end_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
