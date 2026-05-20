from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _ring_x(outer_radius: float, inner_radius: float, width: float) -> cq.Workplane:
    """Hollow cylinder centered on the origin with its axis along local X."""
    outer = cq.Solid.makeCylinder(
        outer_radius,
        width,
        cq.Vector(-width / 2.0, 0.0, 0.0),
        cq.Vector(1.0, 0.0, 0.0),
    )
    inner = cq.Solid.makeCylinder(
        inner_radius,
        width * 1.25,
        cq.Vector(-width * 0.625, 0.0, 0.0),
        cq.Vector(1.0, 0.0, 0.0),
    )
    return cq.Workplane("XY").add(outer.cut(inner))


def _pivot_collar_body() -> cq.Workplane:
    """A square forged collar with a real central clearance for the pivot pin."""
    inner = 0.066
    outer = 0.132
    width = 0.16
    rail = (outer - inner)
    zc = (outer + inner) / 2.0
    top = _box((width, outer * 2.0, rail), (0.0, 0.0, zc))
    bottom = _box((width, outer * 2.0, rail), (0.0, 0.0, -zc))
    side_a = _box((width, rail, outer * 2.0), (0.0, zc, 0.0))
    side_b = _box((width, rail, outer * 2.0), (0.0, -zc, 0.0))
    return top.union(bottom).union(side_a).union(side_b)


def _box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    rotate_x_deg: float = 0.0,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)
    if rotate_x_deg:
        shape = shape.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), rotate_x_deg)
    return shape


def _waterwheel_body() -> cq.Workplane:
    """Broad wooden waterwheel with a thick annular rim and deep outer buckets."""
    outer_r = 1.10
    inner_r = 0.84
    wheel_w = 0.50

    wheel = _ring_x(outer_r, inner_r, wheel_w)
    wheel = wheel.union(_ring_x(0.24, 0.105, 0.44))

    # Two raised side hoops make the rim read broad and built-up rather than flat.
    for x in (-0.215, 0.215):
        hoop = _ring_x(1.15, 1.01, 0.07).translate((x, 0.0, 0.0))
        wheel = wheel.union(hoop)

    # Radial spokes from the bored hub into the rim.
    for i in range(8):
        theta = 2.0 * math.pi * i / 8.0
        spoke = _box(
            (0.12, 0.075, 0.72),
            (0.0, 0.0, 0.54),
            rotate_x_deg=-math.degrees(theta),
        )
        wheel = wheel.union(spoke)

    # Deep buckets: U-shaped troughs around the outside of the wheel, open
    # radially outward and spanning nearly the full wheel width.
    bucket_count = 12
    for i in range(bucket_count):
        theta = 2.0 * math.pi * i / bucket_count
        rot = -math.degrees(theta)
        bottom = _box((0.44, 0.34, 0.045), (0.0, 0.0, 0.955), rotate_x_deg=rot)
        leading_wall = _box(
            (0.44, 0.045, 0.25),
            (0.0, -0.185, 1.075),
            rotate_x_deg=rot,
        )
        trailing_wall = _box(
            (0.44, 0.045, 0.25),
            (0.0, 0.185, 1.075),
            rotate_x_deg=rot,
        )
        outer_lip = _box((0.44, 0.42, 0.04), (0.0, 0.0, 1.205), rotate_x_deg=rot)
        wheel = wheel.union(bottom).union(leading_wall).union(trailing_wall).union(outer_lip)

    return wheel.combine()


def _brake_arm_body() -> cq.Workplane:
    """Brake lever body welded to the outside of the hollow pivot collar."""
    shoe_vector_yz = (0.0, 0.62)
    angle = math.degrees(math.atan2(-shoe_vector_yz[0], shoe_vector_yz[1]))

    arm = _box(
        (0.10, 0.055, 0.52),
        (-0.10, shoe_vector_yz[0] * 0.58, shoe_vector_yz[1] * 0.58),
        rotate_x_deg=angle,
    )
    handle = _box((0.06, 0.05, 0.40), (0.035, 0.0, -0.31), rotate_x_deg=angle)
    grip = _box((0.16, 0.07, 0.055), (0.035, 0.0, -0.535), rotate_x_deg=angle)

    return arm.union(handle).union(grip).combine()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mill_waterwheel_brake")

    wood = Material("weathered_oak", rgba=(0.47, 0.29, 0.14, 1.0))
    dark_wood = Material("wet_dark_wood", rgba=(0.22, 0.13, 0.07, 1.0))
    steel = Material("dark_forged_steel", rgba=(0.10, 0.10, 0.095, 1.0))
    rubber = Material("black_brake_lining", rgba=(0.015, 0.014, 0.013, 1.0))
    stone = Material("stone_foundation", rgba=(0.36, 0.35, 0.32, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((1.95, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.55, 0.06)),
        material=stone,
        name="front_sill",
    )
    support_frame.visual(
        Box((1.95, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.55, 0.06)),
        material=stone,
        name="rear_sill",
    )
    for x, suffix in ((-0.78, "0"), (0.78, "1")):
        support_frame.visual(
            Box((0.13, 1.37, 0.13)),
            origin=Origin(xyz=(x, -0.275, 0.72), rpy=(math.atan2(1.25, 0.55), 0.0, 0.0)),
            material=wood,
            name=f"front_post_{suffix}",
        )
        support_frame.visual(
            Box((0.13, 1.37, 0.13)),
            origin=Origin(xyz=(x, 0.275, 0.72), rpy=(math.atan2(1.25, -0.55), 0.0, 0.0)),
            material=wood,
            name=f"rear_post_{suffix}",
        )
        support_frame.visual(
            Box((0.24, 0.24, 0.18)),
            origin=Origin(xyz=(x, 0.0, 1.35)),
            material=steel,
            name=f"bearing_block_{suffix}",
        )

    support_frame.visual(
        Cylinder(radius=0.108, length=1.86),
        origin=Origin(xyz=(0.0, 0.0, 1.35), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="main_axle",
    )

    # A side bracket and pin carry the separate brake lever.  The cap at the
    # inner end keeps the hollow lever collar visibly clipped on the pivot.
    support_frame.visual(
        Box((0.14, 0.14, 0.86)),
        origin=Origin(xyz=(0.76, -0.70, 0.43)),
        material=wood,
        name="brake_stanchion",
    )
    support_frame.visual(
        Box((0.14, 0.32, 0.10)),
        origin=Origin(xyz=(0.76, -0.62, 0.10)),
        material=wood,
        name="brake_foot",
    )
    support_frame.visual(
        Cylinder(radius=0.066, length=0.42),
        origin=Origin(xyz=(0.60, -0.70, 0.85), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="brake_pivot_pin",
    )
    support_frame.visual(
        Cylinder(radius=0.078, length=0.035),
        origin=Origin(xyz=(0.445, -0.70, 0.85), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="retaining_cap",
    )

    waterwheel = model.part("waterwheel")
    waterwheel.visual(
        mesh_from_cadquery(_waterwheel_body(), "waterwheel_body", tolerance=0.002),
        material=dark_wood,
        name="wheel_body",
    )

    brake_arm = model.part("brake_arm")
    brake_arm.visual(
        mesh_from_cadquery(_pivot_collar_body(), "brake_pivot_collar", tolerance=0.001),
        material=steel,
        name="pivot_collar",
    )
    brake_arm.visual(
        mesh_from_cadquery(_brake_arm_body(), "brake_arm_body", tolerance=0.0015),
        material=steel,
        name="brake_lever",
    )
    brake_arm.visual(
        Box((0.20, 0.11, 0.17)),
        origin=Origin(xyz=(-0.20, 0.0, 0.62)),
        material=rubber,
        name="brake_shoe",
    )

    model.articulation(
        "axle_spin",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=waterwheel,
        origin=Origin(xyz=(0.0, 0.0, 1.35)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4),
    )
    model.articulation(
        "brake_pivot",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=brake_arm,
        origin=Origin(xyz=(0.56, -0.70, 0.85)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=0.42),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    support_frame = object_model.get_part("support_frame")
    waterwheel = object_model.get_part("waterwheel")
    brake_arm = object_model.get_part("brake_arm")
    spin = object_model.get_articulation("axle_spin")
    brake = object_model.get_articulation("brake_pivot")

    ctx.allow_overlap(
        support_frame,
        waterwheel,
        elem_a="main_axle",
        elem_b="wheel_body",
        reason="The stationary axle is intentionally modeled as a captured bearing shaft passing through the wheel hub.",
    )

    ctx.check(
        "waterwheel has continuous axle rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "brake lever has limited pivot swing",
        brake.motion_limits is not None
        and brake.motion_limits.lower == 0.0
        and 0.35 <= brake.motion_limits.upper <= 0.55,
        details=f"limits={brake.motion_limits}",
    )

    ctx.expect_overlap(
        waterwheel,
        support_frame,
        axes="x",
        elem_a="wheel_body",
        elem_b="main_axle",
        min_overlap=0.45,
        name="wheel is carried on the axle span",
    )
    ctx.expect_overlap(
        brake_arm,
        support_frame,
        axes="yz",
        elem_a="pivot_collar",
        elem_b="brake_pivot_pin",
        min_overlap=0.09,
        name="brake collar stays clipped over pivot pin",
    )

    rest_aabb = ctx.part_element_world_aabb(brake_arm, elem="brake_shoe")
    with ctx.pose({brake: brake.motion_limits.upper}):
        closed_aabb = ctx.part_element_world_aabb(brake_arm, elem="brake_shoe")
        ctx.expect_gap(
            brake_arm,
            waterwheel,
            axis="x",
            positive_elem="brake_shoe",
            negative_elem="wheel_body",
            max_gap=0.012,
            max_penetration=0.003,
            name="brake shoe reaches the side face of the rim",
        )

    if rest_aabb is not None and closed_aabb is not None:
        rest_y = (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
        closed_y = (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
        ctx.check(
            "brake swing moves shoe toward the rim",
            closed_y < rest_y - 0.12,
            details=f"rest_y={rest_y:.3f}, closed_y={closed_y:.3f}",
        )
    else:
        ctx.fail("brake shoe pose could be measured", "missing brake_shoe AABB")

    return ctx.report()


object_model = build_object_model()
