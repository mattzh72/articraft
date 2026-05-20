from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _ring_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=36),
            [_circle_profile(inner_radius, segments=36)],
            length,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_latch_pedestrian_gate")

    powder_black = model.material("powder_black", rgba=(0.18, 0.19, 0.20, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    latch_black = model.material("latch_black", rgba=(0.10, 0.11, 0.12, 1.0))
    strike_steel = model.material("strike_steel", rgba=(0.78, 0.79, 0.80, 1.0))

    lower_hinge_z = 0.18
    upper_hinge_z = 1.12
    gate_bottom_local_z = -0.10
    gate_top_local_z = 1.08
    gate_center_z = 0.5 * (gate_bottom_local_z + gate_top_local_z)
    gate_height = gate_top_local_z - gate_bottom_local_z
    gate_outer_width = 0.96
    stile_width = 0.04
    tube_depth = 0.05
    rail_height = 0.04
    hinge_axis_x = 0.045
    latch_post_x = 1.10

    hinge_sleeve = _ring_mesh(
        "hinge_sleeve",
        outer_radius=0.014,
        inner_radius=0.0095,
        length=0.09,
    )

    fence_support = model.part("fence_support")
    fence_support.inertial = Inertial.from_geometry(
        Box((1.14, 0.10, 1.78)),
        mass=48.0,
        origin=Origin(xyz=(0.54, 0.0, 0.77)),
    )
    fence_support.visual(
        Box((0.06, 0.06, 1.65)),
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
        material=powder_black,
        name="hinge_post",
    )
    fence_support.visual(
        Box((0.06, 0.06, 1.65)),
        origin=Origin(xyz=(latch_post_x, 0.0, 0.825)),
        material=powder_black,
        name="latch_post",
    )
    fence_support.visual(
        Box((1.11, 0.08, 0.12)),
        origin=Origin(xyz=(0.525, 0.0, -0.06)),
        material=powder_black,
        name="grade_beam",
    )
    for hinge_name, hinge_z in (("lower", lower_hinge_z), ("upper", upper_hinge_z)):
        fence_support.visual(
            Box((0.03, 0.01, 0.10)),
            origin=Origin(xyz=(0.03, -0.019, hinge_z)),
            material=galvanized_steel,
            name=f"{hinge_name}_post_bracket",
        )
        fence_support.visual(
            Cylinder(radius=0.008, length=0.09),
            origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_z)),
            material=galvanized_steel,
            name=f"{hinge_name}_pintle_pin",
        )
        fence_support.visual(
            Box((0.03, 0.022, 0.01)),
            origin=Origin(xyz=(0.03, -0.019, hinge_z + 0.05)),
            material=galvanized_steel,
            name=f"{hinge_name}_pin_stem",
        )
    fence_support.visual(
        Box((0.04, 0.05, 0.12)),
        origin=Origin(xyz=(1.05, 0.0, 0.76)),
        material=latch_black,
        name="magnet_housing",
    )
    fence_support.visual(
        Box((0.012, 0.07, 0.16)),
        origin=Origin(xyz=(1.064, 0.0, 0.76)),
        material=galvanized_steel,
        name="magnet_mount_plate",
    )

    gate_frame = model.part("gate_frame")
    gate_frame.inertial = Inertial.from_geometry(
        Box((gate_outer_width, tube_depth, gate_height)),
        mass=22.0,
        origin=Origin(xyz=(0.48, 0.0, gate_center_z)),
    )
    gate_frame.visual(
        Box((stile_width, tube_depth, gate_height)),
        origin=Origin(xyz=(0.05, 0.0, gate_center_z)),
        material=powder_black,
        name="hinge_stile",
    )
    gate_frame.visual(
        Box((stile_width, tube_depth, gate_height)),
        origin=Origin(xyz=(0.94, 0.0, gate_center_z)),
        material=powder_black,
        name="latch_stile",
    )
    gate_frame.visual(
        Box((0.92, tube_depth, rail_height)),
        origin=Origin(xyz=(0.49, 0.0, 1.06)),
        material=powder_black,
        name="top_rail",
    )
    gate_frame.visual(
        Box((0.92, tube_depth, rail_height)),
        origin=Origin(xyz=(0.49, 0.0, -0.08)),
        material=powder_black,
        name="bottom_rail",
    )
    gate_frame.visual(
        Box((0.92, tube_depth, rail_height)),
        origin=Origin(xyz=(0.49, 0.0, 0.57)),
        material=powder_black,
        name="mid_rail",
    )

    diagonal_dx = 0.78
    diagonal_dz = 0.94
    gate_frame.visual(
        Box((1.23, 0.028, 0.024)),
        origin=Origin(
            xyz=(0.47, 0.0, 0.47),
            rpy=(0.0, -atan2(diagonal_dz, diagonal_dx), 0.0),
        ),
        material=powder_black,
        name="diagonal_brace",
    )

    gate_frame.visual(
        hinge_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized_steel,
        name="lower_hinge_sleeve",
    )
    gate_frame.visual(
        Box((0.03, 0.01, 0.10)),
        origin=Origin(xyz=(0.018, 0.017, 0.0)),
        material=galvanized_steel,
        name="lower_hinge_tab",
    )
    gate_frame.visual(
        Box((0.03, 0.01, 0.04)),
        origin=Origin(xyz=(0.018, 0.02, upper_hinge_z - lower_hinge_z + 0.065)),
        material=galvanized_steel,
        name="upper_hinge_tab",
    )
    gate_frame.visual(
        Box((0.015, 0.06, 0.10)),
        origin=Origin(xyz=(0.9675, 0.0, 0.58)),
        material=strike_steel,
        name="strike_plate",
    )
    gate_frame.visual(
        Box((0.03, 0.03, 0.14)),
        origin=Origin(xyz=(0.935, 0.0, 0.58)),
        material=latch_black,
        name="latch_body",
    )

    upper_hinge_sleeve = model.part("upper_hinge_sleeve")
    upper_hinge_sleeve.visual(
        hinge_sleeve,
        material=galvanized_steel,
        name="upper_hinge_sleeve",
    )

    model.articulation(
        "lower_pintle_hinge",
        ArticulationType.REVOLUTE,
        parent=fence_support,
        child=gate_frame,
        origin=Origin(xyz=(hinge_axis_x, 0.0, lower_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.8, lower=0.0, upper=1.5),
    )
    model.articulation(
        "upper_pintle_hinge",
        ArticulationType.REVOLUTE,
        parent=gate_frame,
        child=upper_hinge_sleeve,
        origin=Origin(xyz=(0.0, 0.0, upper_hinge_z - lower_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=0.0, upper=1.5),
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

    fence_support = object_model.get_part("fence_support")
    gate_frame = object_model.get_part("gate_frame")
    upper_hinge_sleeve = object_model.get_part("upper_hinge_sleeve")
    lower_pintle = object_model.get_articulation("lower_pintle_hinge")
    upper_pintle = object_model.get_articulation("upper_pintle_hinge")

    ctx.check(
        "both pintle hinges use vertical axes",
        tuple(lower_pintle.axis) == (0.0, 0.0, 1.0) and tuple(upper_pintle.axis) == (0.0, 0.0, 1.0),
        details=f"lower_axis={lower_pintle.axis}, upper_axis={upper_pintle.axis}",
    )

    lower_axis_pos = ctx.part_world_position(gate_frame)
    upper_axis_pos = ctx.part_world_position(upper_hinge_sleeve)
    ctx.check(
        "pintle hinges are vertically aligned on the post",
        lower_axis_pos is not None
        and upper_axis_pos is not None
        and abs(lower_axis_pos[0] - upper_axis_pos[0]) <= 1e-6
        and abs(lower_axis_pos[1] - upper_axis_pos[1]) <= 1e-6
        and 0.90 <= upper_axis_pos[2] - lower_axis_pos[2] <= 0.98,
        details=f"lower_axis={lower_axis_pos}, upper_axis={upper_axis_pos}",
    )

    ctx.expect_gap(
        fence_support,
        gate_frame,
        axis="x",
        positive_elem="magnet_housing",
        negative_elem="strike_plate",
        min_gap=0.01,
        max_gap=0.03,
        name="strike plate sits close to the magnetic housing when closed",
    )
    ctx.expect_overlap(
        fence_support,
        gate_frame,
        axes="yz",
        elem_a="magnet_housing",
        elem_b="strike_plate",
        min_overlap=0.04,
        name="magnetic housing lines up with the strike plate footprint",
    )

    closed_strike = ctx.part_element_world_aabb(gate_frame, elem="strike_plate")
    with ctx.pose({lower_pintle: 1.0}):
        opened_strike = ctx.part_element_world_aabb(gate_frame, elem="strike_plate")

    closed_strike_center_y = None
    opened_strike_center_y = None
    if closed_strike is not None:
        closed_strike_center_y = 0.5 * (closed_strike[0][1] + closed_strike[1][1])
    if opened_strike is not None:
        opened_strike_center_y = 0.5 * (opened_strike[0][1] + opened_strike[1][1])

    ctx.check(
        "gate swings outward toward positive y",
        closed_strike_center_y is not None
        and opened_strike_center_y is not None
        and opened_strike_center_y > closed_strike_center_y + 0.70,
        details=f"closed_y={closed_strike_center_y}, opened_y={opened_strike_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
