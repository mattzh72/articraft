from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SPIN_X = (0.0, pi / 2.0, 0.0)
ALONG_Y = (pi / 2.0, 0.0, 0.0)


def _scaled_sphere(name: str, rx: float, ry: float, rz: float):
    return mesh_from_geometry(
        SphereGeometry(1.0, width_segments=36, height_segments=24).scale(rx, ry, rz),
        name,
    )


def _add_hub_visuals(part, side_sign: float, hub_steel, cap_steel) -> None:
    def sx(value: float) -> float:
        return side_sign * value

    part.visual(
        Cylinder(radius=0.062, length=0.030),
        origin=Origin(xyz=(sx(0.015), 0.0, 0.0), rpy=SPIN_X),
        material=hub_steel,
        name="inner_collar",
    )
    part.visual(
        Cylinder(radius=0.110, length=0.110),
        origin=Origin(xyz=(sx(0.078), 0.0, 0.0), rpy=SPIN_X),
        material=hub_steel,
        name="drum_shell",
    )
    part.visual(
        Cylinder(radius=0.145, length=0.018),
        origin=Origin(xyz=(sx(0.139), 0.0, 0.0), rpy=SPIN_X),
        material=hub_steel,
        name="wheel_flange",
    )
    part.visual(
        Cylinder(radius=0.052, length=0.060),
        origin=Origin(xyz=(sx(0.082), 0.0, 0.0), rpy=SPIN_X),
        material=cap_steel,
        name="grease_cap",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.032),
        origin=Origin(xyz=(sx(0.152), 0.0, 0.0), rpy=SPIN_X),
        material=cap_steel,
        name="pilot_nose",
    )

    stud_radius = 0.074
    stud_x = sx(0.154)
    for stud_index in range(5):
        angle = (2.0 * pi * stud_index) / 5.0
        part.visual(
            Cylinder(radius=0.007, length=0.026),
            origin=Origin(
                xyz=(stud_x, cos(angle) * stud_radius, sin(angle) * stud_radius),
                rpy=SPIN_X,
            ),
            material=cap_steel,
            name=f"lug_{stud_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="torque_tube_rear_axle")

    housing_cast = model.material("housing_cast", rgba=(0.24, 0.25, 0.27, 1.0))
    cover_steel = model.material("cover_steel", rgba=(0.34, 0.35, 0.37, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    oily_steel = model.material("oily_steel", rgba=(0.44, 0.45, 0.47, 1.0))

    banjo_center_mesh = _scaled_sphere("banjo_center", 0.205, 0.155, 0.175)
    rear_cover_mesh = _scaled_sphere("rear_cover", 0.155, 0.055, 0.145)
    pinion_bell_mesh = _scaled_sphere("pinion_bell", 0.105, 0.090, 0.105)
    front_ball_mesh = _scaled_sphere("front_ball_housing", 0.082, 0.082, 0.082)

    axle_housing = model.part("axle_housing")
    axle_housing.visual(
        banjo_center_mesh,
        material=housing_cast,
        name="banjo_center",
    )
    axle_housing.visual(
        rear_cover_mesh,
        origin=Origin(xyz=(0.0, 0.122, 0.0)),
        material=cover_steel,
        name="rear_cover",
    )
    axle_housing.visual(
        pinion_bell_mesh,
        origin=Origin(xyz=(0.0, -0.185, 0.040)),
        material=housing_cast,
        name="pinion_bell",
    )
    axle_housing.visual(
        Cylinder(radius=0.073, length=0.035),
        origin=Origin(xyz=(0.0, -0.282, 0.040), rpy=ALONG_Y),
        material=cover_steel,
        name="pinion_collar",
    )
    axle_housing.visual(
        Cylinder(radius=0.055, length=0.860),
        origin=Origin(xyz=(0.0, -0.700, 0.040), rpy=ALONG_Y),
        material=housing_cast,
        name="torque_tube_main",
    )
    axle_housing.visual(
        front_ball_mesh,
        origin=Origin(xyz=(0.0, -1.090, 0.040)),
        material=cover_steel,
        name="front_ball_housing",
    )
    axle_housing.visual(
        Cylinder(radius=0.070, length=0.070),
        origin=Origin(xyz=(0.0, -1.180, 0.040), rpy=ALONG_Y),
        material=cover_steel,
        name="front_coupling_collar",
    )

    axle_housing.visual(
        Cylinder(radius=0.046, length=0.700),
        origin=Origin(xyz=(-0.355, 0.0, 0.0), rpy=SPIN_X),
        material=housing_cast,
        name="left_axle_tube",
    )
    axle_housing.visual(
        Cylinder(radius=0.046, length=0.700),
        origin=Origin(xyz=(0.355, 0.0, 0.0), rpy=SPIN_X),
        material=housing_cast,
        name="right_axle_tube",
    )
    axle_housing.visual(
        Cylinder(radius=0.062, length=0.104),
        origin=Origin(xyz=(-0.657, 0.0, 0.0), rpy=SPIN_X),
        material=cover_steel,
        name="left_bearing_barrel",
    )
    axle_housing.visual(
        Cylinder(radius=0.062, length=0.104),
        origin=Origin(xyz=(0.657, 0.0, 0.0), rpy=SPIN_X),
        material=cover_steel,
        name="right_bearing_barrel",
    )
    axle_housing.visual(
        Cylinder(radius=0.085, length=0.024),
        origin=Origin(xyz=(-0.717, 0.0, 0.0), rpy=SPIN_X),
        material=cover_steel,
        name="left_end_flange",
    )
    axle_housing.visual(
        Cylinder(radius=0.085, length=0.024),
        origin=Origin(xyz=(0.717, 0.0, 0.0), rpy=SPIN_X),
        material=cover_steel,
        name="right_end_flange",
    )

    cover_bolt_radius_x = 0.118
    cover_bolt_radius_z = 0.106
    for bolt_index in range(10):
        angle = (2.0 * pi * bolt_index) / 10.0
        axle_housing.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(
                xyz=(
                    cos(angle) * cover_bolt_radius_x,
                    0.164,
                    sin(angle) * cover_bolt_radius_z,
                ),
                rpy=ALONG_Y,
            ),
            material=machined_steel,
            name=f"rear_cover_bolt_{bolt_index}",
        )
    axle_housing.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.0, 0.165, -0.120), rpy=ALONG_Y),
        material=machined_steel,
        name="drain_plug",
    )

    left_hub = model.part("left_hub")
    _add_hub_visuals(left_hub, -1.0, machined_steel, oily_steel)

    right_hub = model.part("right_hub")
    _add_hub_visuals(right_hub, 1.0, machined_steel, oily_steel)

    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=axle_housing,
        child=left_hub,
        origin=Origin(xyz=(-0.729, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=35.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=axle_housing,
        child=right_hub,
        origin=Origin(xyz=(0.729, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    axle_housing = object_model.get_part("axle_housing")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")
    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")

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

    ctx.check(
        "hub joints spin about the axle axis",
        tuple(left_hub_spin.axis) == (1.0, 0.0, 0.0) and tuple(right_hub_spin.axis) == (1.0, 0.0, 0.0),
        details=f"left={left_hub_spin.axis}, right={right_hub_spin.axis}",
    )

    left_type = getattr(left_hub_spin.articulation_type, "name", str(left_hub_spin.articulation_type))
    right_type = getattr(right_hub_spin.articulation_type, "name", str(right_hub_spin.articulation_type))
    ctx.check(
        "hub joints are rotational wheel joints",
        left_type in {"CONTINUOUS", "REVOLUTE", "ArticulationType.CONTINUOUS", "ArticulationType.REVOLUTE"}
        and right_type in {"CONTINUOUS", "REVOLUTE", "ArticulationType.CONTINUOUS", "ArticulationType.REVOLUTE"},
        details=f"left={left_type}, right={right_type}",
    )

    ctx.expect_gap(
        axle_housing,
        left_hub,
        axis="x",
        positive_elem="left_end_flange",
        negative_elem="inner_collar",
        max_gap=0.0005,
        max_penetration=0.0,
        name="left hub seats on left flange face",
    )
    ctx.expect_gap(
        right_hub,
        axle_housing,
        axis="x",
        positive_elem="inner_collar",
        negative_elem="right_end_flange",
        max_gap=0.0005,
        max_penetration=0.0,
        name="right hub seats on right flange face",
    )
    ctx.expect_overlap(
        left_hub,
        axle_housing,
        axes="yz",
        elem_a="inner_collar",
        elem_b="left_end_flange",
        min_overlap=0.11,
        name="left hub collar stays centered on the bearing flange",
    )
    ctx.expect_overlap(
        right_hub,
        axle_housing,
        axes="yz",
        elem_a="inner_collar",
        elem_b="right_end_flange",
        min_overlap=0.11,
        name="right hub collar stays centered on the bearing flange",
    )

    left_pos = ctx.part_world_position(left_hub)
    right_pos = ctx.part_world_position(right_hub)
    if left_pos is None or right_pos is None:
        ctx.fail("hub world positions resolve", "Could not resolve left/right hub world positions.")
    else:
        ctx.check(
            "hubs are symmetric about the differential centerline",
            abs(left_pos[0] + right_pos[0]) < 1e-6
            and abs(left_pos[1] - right_pos[1]) < 1e-6
            and abs(left_pos[2] - right_pos[2]) < 1e-6,
            details=f"left={left_pos}, right={right_pos}",
        )

    with ctx.pose({left_hub_spin: pi / 5.0, right_hub_spin: -pi / 7.0}):
        ctx.expect_gap(
            axle_housing,
            left_hub,
            axis="x",
            positive_elem="left_end_flange",
            negative_elem="inner_collar",
            max_gap=0.0005,
            max_penetration=0.0,
            name="left hub stays seated while spun",
        )
        ctx.expect_gap(
            right_hub,
            axle_housing,
            axis="x",
            positive_elem="inner_collar",
            negative_elem="right_end_flange",
            max_gap=0.0005,
            max_penetration=0.0,
            name="right hub stays seated while spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
