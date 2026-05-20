from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    black_powder = model.material("black_powder", rgba=(0.11, 0.11, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.82, 0.83, 0.85, 1.0))
    charcoal_vinyl = model.material("charcoal_vinyl", rgba=(0.16, 0.16, 0.17, 1.0))

    pedestal_base_shell = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.12, 0.0),
                (0.19, 0.004),
                (0.212, 0.012),
                (0.220, 0.024),
                (0.213, 0.032),
                (0.120, 0.041),
                (0.050, 0.051),
                (0.038, 0.068),
                (0.0, 0.068),
            ],
            segments=72,
        ),
        "pedestal_base_shell",
    )
    foot_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.145, tube=0.014, radial_segments=18, tubular_segments=72),
        "foot_ring",
    )
    left_support_rail = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.032, 0.020),
                (0.012, 0.042, 0.027),
                (0.026, 0.052, 0.031),
            ],
            radius=0.008,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "left_support_rail",
    )
    right_support_rail = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, -0.032, 0.020),
                (0.012, -0.042, 0.027),
                (0.026, -0.052, 0.031),
            ],
            radius=0.008,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "right_support_rail",
    )
    cushion_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, -0.022),
                (0.08, -0.022),
                (0.155, -0.018),
                (0.176, -0.008),
                (0.183, 0.008),
                (0.174, 0.023),
                (0.110, 0.032),
                (0.0, 0.026),
            ],
            segments=72,
        ),
        "seat_cushion",
    )

    base = model.part("base")
    base.visual(pedestal_base_shell, material=black_powder, name="floor_base")
    base.visual(
        Cylinder(radius=0.032, length=0.582),
        origin=Origin(xyz=(0.0, 0.0, 0.359)),
        material=brushed_steel,
        name="seat_post",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
        material=satin_aluminum,
        name="lower_post_shroud",
    )
    base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=brushed_steel,
        name="foot_ring",
    )
    for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        base.visual(
            Box((0.120, 0.018, 0.018)),
            origin=Origin(xyz=(0.086, 0.0, 0.315), rpy=(0.0, 0.0, yaw)),
            material=brushed_steel,
            name=f"foot_ring_spoke_{index}",
        )
    base.visual(
        Cylinder(radius=0.055, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.672)),
        material=satin_aluminum,
        name="bearing_collar",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.44, 0.44, 0.694)),
        mass=17.0,
        origin=Origin(xyz=(0.0, 0.0, 0.347)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black_powder,
        name="swivel_plate",
    )
    seat.visual(
        Cylinder(radius=0.022, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_aluminum,
        name="swivel_spigot",
    )
    seat.visual(
        Box((0.100, 0.180, 0.012)),
        origin=Origin(xyz=(0.035, 0.0, 0.030)),
        material=black_powder,
        name="underseat_bracket",
    )
    seat.visual(left_support_rail, material=satin_aluminum, name="left_support_rail")
    seat.visual(right_support_rail, material=satin_aluminum, name="right_support_rail")
    seat.visual(
        Cylinder(radius=0.172, length=0.020),
        origin=Origin(xyz=(0.035, 0.0, 0.046)),
        material=black_powder,
        name="seat_pan",
    )
    seat.visual(
        cushion_mesh,
        origin=Origin(xyz=(0.035, 0.0, 0.078)),
        material=charcoal_vinyl,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.38, 0.38, 0.110)),
        mass=5.0,
        origin=Origin(xyz=(0.035, 0.0, 0.055)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.694)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=8.0),
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

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    def aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    with ctx.pose({swivel: 0.0}):
        ctx.expect_gap(
            seat,
            base,
            axis="z",
            positive_elem="swivel_plate",
            negative_elem="bearing_collar",
            max_gap=0.001,
            max_penetration=1e-6,
            name="swivel plate seats on the bearing collar",
        )
        ctx.expect_overlap(
            seat,
            base,
            axes="xy",
            elem_a="swivel_plate",
            elem_b="bearing_collar",
            min_overlap=0.10,
            name="swivel plate remains centered over the pedestal bearing",
        )
        rest_pan_aabb = ctx.part_element_world_aabb(seat, elem="seat_pan")
        rest_cushion_aabb = ctx.part_element_world_aabb(seat, elem="seat_cushion")

    with ctx.pose({swivel: math.pi / 2.0}):
        quarter_pan_aabb = ctx.part_element_world_aabb(seat, elem="seat_pan")

    rest_pan_center = aabb_center(rest_pan_aabb) if rest_pan_aabb is not None else None
    quarter_pan_center = aabb_center(quarter_pan_aabb) if quarter_pan_aabb is not None else None

    ctx.check(
        "seat pan is laterally offset from the pedestal at rest",
        rest_pan_center is not None and rest_pan_center[0] > 0.025 and abs(rest_pan_center[1]) < 0.02,
        details=f"rest_pan_center={rest_pan_center}",
    )
    ctx.check(
        "seat offset rotates around the vertical pedestal axis",
        quarter_pan_center is not None and quarter_pan_center[1] > 0.025 and abs(quarter_pan_center[0]) < 0.02,
        details=f"quarter_pan_center={quarter_pan_center}",
    )
    ctx.check(
        "seat height reads as a bar stool",
        rest_cushion_aabb is not None and 0.76 <= rest_cushion_aabb[1][2] <= 0.84,
        details=f"seat_top_z={None if rest_cushion_aabb is None else rest_cushion_aabb[1][2]}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
