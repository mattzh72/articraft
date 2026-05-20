from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


LINK_0_LENGTH = 0.220
LINK_1_LENGTH = 0.180
END_TAB_LENGTH = 0.100
LINK_WIDTH = 0.054
LINK_THICKNESS = 0.012
PIVOT_HOLE_RADIUS = 0.013
PIN_RADIUS = 0.0075
ROOT_Z = 0.030
LINK_1_Z_OFFSET = 0.019
END_TAB_Z_OFFSET = 0.038


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * cos(2.0 * pi * i / segments),
            cy + radius * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(length: float, width: float, segments: int = 24) -> list[tuple[float, float]]:
    """A flat rounded link outline with pivot centers at x=0 and x=length."""
    radius = width / 2.0
    pts: list[tuple[float, float]] = []
    for i in range(segments + 1):
        angle = -pi / 2.0 + pi * i / segments
        pts.append((length + radius * cos(angle), radius * sin(angle)))
    for i in range(segments + 1):
        angle = pi / 2.0 + pi * i / segments
        pts.append((radius * cos(angle), radius * sin(angle)))
    return pts


def _flat_link_mesh(name: str, length: float, *, distal_hole: bool = True):
    holes = [_circle_profile(0.0, 0.0, PIVOT_HOLE_RADIUS)]
    if distal_hole:
        holes.append(_circle_profile(length, 0.0, PIVOT_HOLE_RADIUS))
    geom = ExtrudeWithHolesGeometry(
        _capsule_profile(length, LINK_WIDTH),
        holes,
        LINK_THICKNESS,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def _mounting_plate_mesh():
    holes = [
        _circle_profile(-0.120, -0.060, 0.006),
        _circle_profile(-0.120, 0.060, 0.006),
        _circle_profile(0.040, -0.060, 0.006),
        _circle_profile(0.040, 0.060, 0.006),
    ]
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.280, 0.165, 0.018, corner_segments=8),
        holes,
        0.012,
        center=True,
    )
    geom.translate(-0.045, 0.0, 0.0)
    return mesh_from_geometry(geom, "mounting_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_revolute_chain")

    model.material("blackened_steel", rgba=(0.05, 0.055, 0.060, 1.0))
    model.material("brushed_plate", rgba=(0.42, 0.44, 0.46, 1.0))
    model.material("blue_anodized", rgba=(0.12, 0.24, 0.42, 1.0))
    model.material("clear_anodized", rgba=(0.62, 0.65, 0.68, 1.0))
    model.material("dark_bushing", rgba=(0.10, 0.105, 0.11, 1.0))
    model.material("screw_dark", rgba=(0.025, 0.026, 0.028, 1.0))

    base = model.part("base")
    base.visual(_mounting_plate_mesh(), origin=Origin(xyz=(0.0, 0.0, 0.006)), material="brushed_plate", name="mounting_plate")
    base.visual(Box((0.072, 0.072, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.017)), material="blackened_steel", name="root_boss")
    base.visual(Cylinder(radius=0.026, length=0.010), origin=Origin(xyz=(0.0, 0.0, 0.021)), material="dark_bushing", name="root_bearing")
    base.visual(Cylinder(radius=PIN_RADIUS, length=0.036), origin=Origin(xyz=(0.0, 0.0, 0.030)), material="blackened_steel", name="root_pin")
    base.visual(Cylinder(radius=0.018, length=0.004), origin=Origin(xyz=(0.0, 0.0, 0.040)), material="blackened_steel", name="root_pin_cap")
    for x_pos in (-0.120, 0.040):
        for y_pos in (-0.060, 0.060):
            base.visual(
                Cylinder(radius=0.011, length=0.003),
                origin=Origin(xyz=(x_pos - 0.045, y_pos, 0.0125)),
                material="screw_dark",
                name=f"mount_screw_{x_pos}_{y_pos}",
            )

    link_0 = model.part("link_0")
    link_0.visual(_flat_link_mesh("link_0_plate", LINK_0_LENGTH), material="blue_anodized", name="link_0_plate")
    link_0.visual(Cylinder(radius=PIN_RADIUS, length=0.038), origin=Origin(xyz=(LINK_0_LENGTH, 0.0, 0.013)), material="blackened_steel", name="joint_2_pin")
    link_0.visual(Cylinder(radius=0.018, length=0.004), origin=Origin(xyz=(LINK_0_LENGTH, 0.0, -0.008)), material="blackened_steel", name="joint_2_lower_cap")
    link_0.visual(Cylinder(radius=0.016, length=0.004), origin=Origin(xyz=(LINK_0_LENGTH, 0.0, 0.029)), material="blackened_steel", name="joint_2_upper_cap")

    link_1 = model.part("link_1")
    link_1.visual(_flat_link_mesh("link_1_plate", LINK_1_LENGTH), origin=Origin(xyz=(0.0, 0.0, LINK_1_Z_OFFSET)), material="clear_anodized", name="link_1_plate")
    link_1.visual(Cylinder(radius=PIN_RADIUS, length=0.038), origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.032)), material="blackened_steel", name="joint_3_pin")
    link_1.visual(Cylinder(radius=0.018, length=0.004), origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.011)), material="blackened_steel", name="joint_3_lower_cap")
    link_1.visual(Cylinder(radius=0.016, length=0.004), origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.048)), material="blackened_steel", name="joint_3_upper_cap")

    end_tab = model.part("end_tab")
    end_tab.visual(
        _flat_link_mesh("end_tab_plate", END_TAB_LENGTH, distal_hole=False),
        origin=Origin(xyz=(0.0, 0.0, END_TAB_Z_OFFSET)),
        material="blue_anodized",
        name="end_tab_plate",
    )
    end_tab.visual(
        Cylinder(radius=0.007, length=0.002),
        origin=Origin(xyz=(END_TAB_LENGTH, 0.0, END_TAB_Z_OFFSET + LINK_THICKNESS / 2.0 + 0.001)),
        material="screw_dark",
        name="tab_fastener_hole",
    )

    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, ROOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_0_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.4, lower=-1.65, upper=1.65),
    )
    model.articulation(
        "link_1_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=end_tab,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.6, lower=-1.75, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    end_tab = object_model.get_part("end_tab")
    joints = [object_model.get_articulation(name) for name in ("base_to_link_0", "link_0_to_link_1", "link_1_to_end_tab")]

    ctx.allow_overlap(
        base,
        link_0,
        elem_a="root_pin",
        elem_b="link_0_plate",
        reason="The root pivot pin is intentionally modeled as passing through the clearance bore of the first flat link.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="joint_2_pin",
        elem_b="link_1_plate",
        reason="The second pivot pin is intentionally captured through the clearance bore of the upper stacked link.",
    )
    ctx.allow_overlap(
        link_1,
        end_tab,
        elem_a="joint_3_pin",
        elem_b="end_tab_plate",
        reason="The distal pivot pin intentionally passes through the clearance bore of the compact end tab.",
    )

    ctx.check(
        "three vertical revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(abs(j.axis[0]) < 1e-9 and abs(j.axis[1]) < 1e-9 and abs(j.axis[2] - 1.0) < 1e-9 for j in joints),
        details=f"joints={[(j.name, j.articulation_type, j.axis) for j in object_model.articulations]}",
    )
    ctx.expect_gap(
        link_0,
        base,
        axis="z",
        positive_elem="link_0_plate",
        negative_elem="mounting_plate",
        min_gap=0.010,
        max_gap=0.016,
        name="root link clears the mounting plate",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="z",
        positive_elem="link_1_plate",
        negative_elem="link_0_plate",
        min_gap=0.005,
        max_gap=0.010,
        name="second link is stacked just above the first link",
    )
    ctx.expect_gap(
        end_tab,
        link_1,
        axis="z",
        positive_elem="end_tab_plate",
        negative_elem="link_1_plate",
        min_gap=0.005,
        max_gap=0.010,
        name="end tab is stacked just above the second link",
    )
    ctx.expect_overlap(base, link_0, axes="z", elem_a="root_pin", elem_b="link_0_plate", min_overlap=0.010, name="root pin spans the first link thickness")
    ctx.expect_overlap(link_0, link_1, axes="z", elem_a="joint_2_pin", elem_b="link_1_plate", min_overlap=0.010, name="second pin spans the upper link thickness")
    ctx.expect_overlap(link_1, end_tab, axes="z", elem_a="joint_3_pin", elem_b="end_tab_plate", min_overlap=0.010, name="distal pin spans the end tab thickness")
    ctx.expect_overlap(link_0, link_1, axes="xy", elem_a="link_0_plate", elem_b="link_1_plate", min_overlap=0.020, name="middle joint knuckles share a pivot footprint")
    ctx.expect_overlap(link_1, end_tab, axes="xy", elem_a="link_1_plate", elem_b="end_tab_plate", min_overlap=0.020, name="distal joint knuckles share a pivot footprint")

    rest_end = ctx.part_world_position(end_tab)
    with ctx.pose({"base_to_link_0": 0.85, "link_0_to_link_1": 0.55, "link_1_to_end_tab": -0.35}):
        bent_end = ctx.part_world_position(end_tab)
    ctx.check(
        "chain bends in the horizontal plane",
        rest_end is not None
        and bent_end is not None
        and bent_end[1] > rest_end[1] + 0.20
        and abs(bent_end[2] - rest_end[2]) < 1e-6,
        details=f"rest={rest_end}, bent={bent_end}",
    )

    return ctx.report()


object_model = build_object_model()
