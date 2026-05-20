from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_RADIUS = 0.18
FOOT_HEIGHT = 0.022
PEDESTAL_LOWER_RADIUS = 0.075
PEDESTAL_LOWER_HEIGHT = 0.09
PEDESTAL_UPPER_RADIUS = 0.055
PEDESTAL_UPPER_HEIGHT = 0.095

SHAFT_RADIUS = 0.022
SHAFT_BASE_Z = FOOT_HEIGHT + PEDESTAL_LOWER_HEIGHT + PEDESTAL_UPPER_HEIGHT
SHAFT_HEIGHT = 0.332

SEAT_THICKNESS = 0.006
CAP_RADIUS = 0.032
CAP_HEIGHT = 0.014
CAP_BASE_Z = SHAFT_BASE_Z + SHAFT_HEIGHT

BORE_RADIUS = 0.023

CARRIER_SPECS = (
    {
        "name": "lower_carrier",
        "joint_name": "base_to_lower_carrier",
        "joint_z": 0.238,
        "outer_radius": 0.155,
        "hub_outer_radius": 0.05,
        "sleeve_height": 0.042,
        "tray_thickness": 0.01,
        "rim_width": 0.018,
        "rim_height": 0.012,
        "seat_radius": 0.034,
        "knob_radius": 0.012,
        "knob_height": 0.026,
        "knob_angle_deg": 0.0,
    },
    {
        "name": "middle_carrier",
        "joint_name": "base_to_middle_carrier",
        "joint_z": 0.338,
        "outer_radius": 0.118,
        "hub_outer_radius": 0.046,
        "sleeve_height": 0.039,
        "tray_thickness": 0.01,
        "rim_width": 0.016,
        "rim_height": 0.011,
        "seat_radius": 0.031,
        "knob_radius": 0.01,
        "knob_height": 0.024,
        "knob_angle_deg": 142.0,
    },
    {
        "name": "upper_carrier",
        "joint_name": "base_to_upper_carrier",
        "joint_z": 0.438,
        "outer_radius": 0.087,
        "hub_outer_radius": 0.042,
        "sleeve_height": 0.036,
        "tray_thickness": 0.009,
        "rim_width": 0.014,
        "rim_height": 0.01,
        "seat_radius": 0.029,
        "knob_radius": 0.009,
        "knob_height": 0.022,
        "knob_angle_deg": -108.0,
    },
)


def _build_base_body() -> cq.Workplane:
    foot = cq.Workplane("XY").circle(FOOT_RADIUS).extrude(FOOT_HEIGHT)
    lower_pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_LOWER_RADIUS)
        .extrude(PEDESTAL_LOWER_HEIGHT)
        .translate((0.0, 0.0, FOOT_HEIGHT))
    )
    upper_pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_UPPER_RADIUS)
        .extrude(PEDESTAL_UPPER_HEIGHT)
        .translate((0.0, 0.0, FOOT_HEIGHT + PEDESTAL_LOWER_HEIGHT))
    )
    return foot.union(lower_pedestal).union(upper_pedestal)


def _build_carrier_shape(
    *,
    outer_radius: float,
    hub_outer_radius: float,
    sleeve_height: float,
    tray_thickness: float,
    rim_width: float,
    rim_height: float,
) -> cq.Workplane:
    sleeve = cq.Workplane("XY").circle(hub_outer_radius).circle(BORE_RADIUS).extrude(sleeve_height)
    tray = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(BORE_RADIUS)
        .extrude(tray_thickness)
        .translate((0.0, 0.0, sleeve_height - tray_thickness))
    )
    rim = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(outer_radius - rim_width)
        .extrude(rim_height)
        .translate((0.0, 0.0, sleeve_height))
    )
    return sleeve.union(tray).union(rim)


def _center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_rotary_fixture")

    base_finish = model.material("base_finish", color=(0.22, 0.23, 0.25, 1.0))
    shaft_finish = model.material("shaft_finish", color=(0.68, 0.7, 0.72, 1.0))
    carrier_finish = model.material("carrier_finish", color=(0.72, 0.74, 0.76, 1.0))
    knob_finish = model.material("knob_finish", color=(0.1, 0.1, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_body(), "base_body"),
        material=base_finish,
        name="base_body",
    )
    base.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_BASE_Z + SHAFT_HEIGHT * 0.5)),
        material=shaft_finish,
        name="spindle",
    )
    for spec in CARRIER_SPECS:
        seat_name = spec["name"].replace("_carrier", "_seat")
        base.visual(
            Cylinder(radius=spec["seat_radius"], length=SEAT_THICKNESS),
            origin=Origin(xyz=(0.0, 0.0, spec["joint_z"] - SEAT_THICKNESS * 0.5)),
            material=shaft_finish,
            name=seat_name,
        )
    base.visual(
        Cylinder(radius=CAP_RADIUS, length=CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CAP_BASE_Z + CAP_HEIGHT * 0.5)),
        material=shaft_finish,
        name="retainer_cap",
    )

    limits = MotionLimits(
        effort=12.0,
        velocity=2.5,
        lower=-math.pi,
        upper=math.pi,
    )

    for spec in CARRIER_SPECS:
        carrier = model.part(spec["name"])
        carrier.visual(
            mesh_from_cadquery(
                _build_carrier_shape(
                    outer_radius=spec["outer_radius"],
                    hub_outer_radius=spec["hub_outer_radius"],
                    sleeve_height=spec["sleeve_height"],
                    tray_thickness=spec["tray_thickness"],
                    rim_width=spec["rim_width"],
                    rim_height=spec["rim_height"],
                ),
                f"{spec['name']}_shell",
            ),
            material=carrier_finish,
            name=f"{spec['name']}_shell",
        )

        knob_radius_xy = spec["outer_radius"] - spec["rim_width"] * 0.85
        knob_angle = math.radians(spec["knob_angle_deg"])
        knob_embed = 0.001
        knob_center_z = spec["sleeve_height"] - knob_embed + spec["knob_height"] * 0.5
        carrier.visual(
            Cylinder(radius=spec["knob_radius"], length=spec["knob_height"]),
            origin=Origin(
                xyz=(
                    knob_radius_xy * math.cos(knob_angle),
                    knob_radius_xy * math.sin(knob_angle),
                    knob_center_z,
                )
            ),
            material=knob_finish,
            name=spec["name"].replace("_carrier", "_knob"),
        )

        model.articulation(
            spec["joint_name"],
            ArticulationType.REVOLUTE,
            parent=base,
            child=carrier,
            origin=Origin(xyz=(0.0, 0.0, spec["joint_z"])),
            axis=(0.0, 0.0, 1.0),
            motion_limits=limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base")
    lower = object_model.get_part("lower_carrier")
    middle = object_model.get_part("middle_carrier")
    upper = object_model.get_part("upper_carrier")

    lower_joint = object_model.get_articulation("base_to_lower_carrier")
    middle_joint = object_model.get_articulation("base_to_middle_carrier")
    upper_joint = object_model.get_articulation("base_to_upper_carrier")

    expected_z = {
        "base_to_lower_carrier": 0.238,
        "base_to_middle_carrier": 0.338,
        "base_to_upper_carrier": 0.438,
    }
    for joint in (lower_joint, middle_joint, upper_joint):
        axis_ok = tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0)
        origin = joint.origin.xyz
        coaxial_ok = (
            abs(origin[0]) < 1e-6
            and abs(origin[1]) < 1e-6
            and abs(origin[2] - expected_z[joint.name]) < 1e-6
        )
        limits = joint.motion_limits
        limit_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
        )
        ctx.check(
            f"{joint.name} uses the common vertical spindle axis",
            axis_ok and coaxial_ok and limit_ok,
            f"axis={joint.axis}, origin={origin}, limits={limits}",
        )

    ctx.expect_contact(
        lower,
        base,
        elem_b="lower_seat",
        contact_tol=0.0015,
        name="lower carrier is seated on its thrust collar",
    )
    ctx.expect_contact(
        middle,
        base,
        elem_b="middle_seat",
        contact_tol=0.0015,
        name="middle carrier is seated on its thrust collar",
    )
    ctx.expect_contact(
        upper,
        base,
        elem_b="upper_seat",
        contact_tol=0.0015,
        name="upper carrier is seated on its thrust collar",
    )

    ctx.expect_origin_gap(
        middle,
        lower,
        axis="z",
        min_gap=0.095,
        max_gap=0.105,
        name="middle carrier is stepped above the lower carrier",
    )
    ctx.expect_origin_gap(
        upper,
        middle,
        axis="z",
        min_gap=0.095,
        max_gap=0.105,
        name="upper carrier is stepped above the middle carrier",
    )
    ctx.expect_gap(
        middle,
        lower,
        axis="z",
        min_gap=0.03,
        name="middle carrier clears the lower carrier",
    )
    ctx.expect_gap(
        upper,
        middle,
        axis="z",
        min_gap=0.03,
        name="upper carrier clears the middle carrier",
    )

    lower_knob_rest = _center_from_aabb(ctx.part_element_world_aabb(lower, elem="lower_knob"))
    middle_knob_rest = _center_from_aabb(ctx.part_element_world_aabb(middle, elem="middle_knob"))
    upper_knob_rest = _center_from_aabb(ctx.part_element_world_aabb(upper, elem="upper_knob"))

    with ctx.pose({lower_joint: 1.1}):
        lower_knob_turned = _center_from_aabb(ctx.part_element_world_aabb(lower, elem="lower_knob"))
        middle_knob_when_lower_turns = _center_from_aabb(ctx.part_element_world_aabb(middle, elem="middle_knob"))

    if (
        lower_knob_rest is not None
        and lower_knob_turned is not None
        and middle_knob_rest is not None
        and middle_knob_when_lower_turns is not None
    ):
        lower_xy_shift = math.hypot(
            lower_knob_turned[0] - lower_knob_rest[0],
            lower_knob_turned[1] - lower_knob_rest[1],
        )
        lower_z_shift = abs(lower_knob_turned[2] - lower_knob_rest[2])
        middle_xy_shift = math.hypot(
            middle_knob_when_lower_turns[0] - middle_knob_rest[0],
            middle_knob_when_lower_turns[1] - middle_knob_rest[1],
        )
        ctx.check(
            "lower carrier knob traces a circular path when rotated",
            lower_xy_shift > 0.08 and lower_z_shift < 0.002,
            f"lower_xy_shift={lower_xy_shift:.4f}, lower_z_shift={lower_z_shift:.4f}",
        )
        ctx.check(
            "lower carrier rotation does not drag the middle carrier",
            middle_xy_shift < 0.002,
            f"middle_xy_shift={middle_xy_shift:.4f}",
        )
    else:
        ctx.fail("carrier knob motion could be measured", "missing knob AABB during lower-carrier pose check")

    with ctx.pose({upper_joint: -0.95}):
        upper_knob_turned = _center_from_aabb(ctx.part_element_world_aabb(upper, elem="upper_knob"))

    if upper_knob_rest is not None and upper_knob_turned is not None:
        upper_xy_shift = math.hypot(
            upper_knob_turned[0] - upper_knob_rest[0],
            upper_knob_turned[1] - upper_knob_rest[1],
        )
        upper_z_shift = abs(upper_knob_turned[2] - upper_knob_rest[2])
        ctx.check(
            "upper carrier rotates independently about the same spindle",
            upper_xy_shift > 0.05 and upper_z_shift < 0.002,
            f"upper_xy_shift={upper_xy_shift:.4f}, upper_z_shift={upper_z_shift:.4f}",
        )
    else:
        ctx.fail("upper knob motion could be measured", "missing knob AABB during upper-carrier pose check")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
