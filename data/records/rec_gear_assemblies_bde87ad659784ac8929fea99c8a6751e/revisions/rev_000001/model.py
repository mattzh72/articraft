from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


HALF_PI = math.pi / 2.0
SHAFT_AXIS_RPY = (-HALF_PI, 0.0, 0.0)

SHAFT_CENTERS = (
    (0.000, 0.360),
    (0.115, 0.313),
    (0.230, 0.266),
    (0.345, 0.219),
)
GEAR_MODULE = 0.0035
SMALL_TEETH = 18
LARGE_TEETH = 48
SMALL_WIDTH = 0.024
LARGE_WIDTH = 0.028
GEAR_PLANE_A = -0.055
GEAR_PLANE_B = 0.055


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _side_plate_geometry() -> ExtrudeWithHolesGeometry:
    # Local X is bench length. Local Y maps to world -Z after the visual rotation,
    # so the profile coordinates are written as (x, -z).
    outer = [
        (-0.085, -0.050),
        (-0.085, -0.485),
        (0.455, -0.485),
        (0.455, -0.050),
    ]
    holes = [_circle_profile(x, -z, 0.026) for x, z in SHAFT_CENTERS]
    return ExtrudeWithHolesGeometry(outer, holes, 0.028, center=True)


def _large_gear():
    return SpurGear(GEAR_MODULE, LARGE_TEETH, LARGE_WIDTH).build(
        bore_d=0.018,
        hub_d=0.054,
        hub_length=0.042,
        n_spokes=6,
        spoke_width=0.008,
        spokes_id=0.058,
        spokes_od=0.135,
        chamfer=0.0008,
    )


def _small_gear():
    return SpurGear(GEAR_MODULE, SMALL_TEETH, SMALL_WIDTH).build(
        bore_d=0.018,
        hub_d=0.040,
        hub_length=0.036,
        chamfer=0.0007,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacked_reduction_gear_bench")

    frame_paint = model.material("painted_frame_blue", rgba=(0.08, 0.18, 0.28, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    bronze = model.material("oiled_bronze_gears", rgba=(0.76, 0.54, 0.22, 1.0))
    gunmetal = model.material("gunmetal_pinions", rgba=(0.36, 0.38, 0.39, 1.0))
    black = model.material("black_bearing_seals", rgba=(0.03, 0.03, 0.035, 1.0))

    frame = model.part("side_frame")
    frame.visual(
        Box((0.64, 0.48, 0.040)),
        origin=Origin(xyz=(0.180, 0.0, 0.020)),
        material=frame_paint,
        name="bench_base",
    )
    frame.visual(
        Box((0.56, 0.028, 0.032)),
        origin=Origin(xyz=(0.185, -0.185, 0.056)),
        material=frame_paint,
        name="front_lower_rail",
    )
    frame.visual(
        Box((0.56, 0.028, 0.032)),
        origin=Origin(xyz=(0.185, 0.185, 0.056)),
        material=frame_paint,
        name="rear_lower_rail",
    )
    frame.visual(
        Box((0.56, 0.028, 0.032)),
        origin=Origin(xyz=(0.185, -0.185, 0.472)),
        material=frame_paint,
        name="front_top_rail",
    )
    frame.visual(
        Box((0.56, 0.028, 0.032)),
        origin=Origin(xyz=(0.185, 0.185, 0.472)),
        material=frame_paint,
        name="rear_top_rail",
    )
    for side, y in (("front", -0.185), ("rear", 0.185)):
        for idx, (x, z) in enumerate(SHAFT_CENTERS):
            web_height = max(0.040, 0.472 - (z + 0.032))
            frame.visual(
                Box((0.020, 0.030, web_height)),
                origin=Origin(xyz=(x, y, z + 0.032 + web_height * 0.5)),
                material=frame_paint,
                name=f"{side}_upper_web_{idx}",
            )
            web_height = max(0.030, (z - 0.032) - 0.056)
            frame.visual(
                Box((0.020, 0.030, web_height)),
                origin=Origin(xyz=(x, y, 0.056 + web_height * 0.5)),
                material=frame_paint,
                name=f"{side}_lower_web_{idx}",
            )
    # Two tie rods make the separated side cheeks read as one grounded frame.
    for i, (x, z) in enumerate(((-0.055, 0.465), (0.425, 0.075))):
        frame.visual(
            Cylinder(radius=0.012, length=0.420),
            origin=Origin(xyz=(x, 0.0, z), rpy=SHAFT_AXIS_RPY),
            material=polished_steel,
            name=f"tie_rod_{i}",
        )
    # Raised bushings around every shaft hole on both side cheeks.
    for side, y in (("front", -0.201), ("rear", 0.201)):
        for idx, (x, z) in enumerate(SHAFT_CENTERS):
            frame.visual(
                mesh_from_geometry(TorusGeometry(0.026, 0.008, radial_segments=32, tubular_segments=12), f"{side}_bearing_{idx}"),
                origin=Origin(xyz=(x, y, z), rpy=SHAFT_AXIS_RPY),
                material=black,
                name=f"{side}_bearing_{idx}",
            )

    large_mesh = mesh_from_cadquery(_large_gear(), "large_spur_gear", tolerance=0.0007, angular_tolerance=0.08)
    small_mesh = mesh_from_cadquery(_small_gear(), "small_spur_gear", tolerance=0.0006, angular_tolerance=0.08)

    gear_layout = {
        0: (("small_gear", small_mesh, GEAR_PLANE_A, gunmetal),),
        1: (
            ("large_gear", large_mesh, GEAR_PLANE_A, bronze),
            ("small_gear", small_mesh, GEAR_PLANE_B, gunmetal),
        ),
        2: (
            ("large_gear", large_mesh, GEAR_PLANE_B, bronze),
            ("small_gear", small_mesh, GEAR_PLANE_A, gunmetal),
        ),
        3: (("large_gear", large_mesh, GEAR_PLANE_A, bronze),),
    }

    for idx, (x, z) in enumerate(SHAFT_CENTERS):
        shaft = model.part(f"shaft_{idx}")
        shaft.visual(
            Cylinder(radius=0.011, length=0.455),
            origin=Origin(rpy=SHAFT_AXIS_RPY),
            material=dark_steel,
            name="axle",
        )
        shaft.visual(
            Cylinder(radius=0.0186, length=0.026),
            origin=Origin(xyz=(0.0, -0.205, 0.0), rpy=SHAFT_AXIS_RPY),
            material=polished_steel,
            name="journal_0",
        )
        shaft.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(xyz=(0.0, -0.225, 0.0), rpy=SHAFT_AXIS_RPY),
            material=dark_steel,
            name="retainer_0",
        )
        shaft.visual(
            Cylinder(radius=0.0186, length=0.026),
            origin=Origin(xyz=(0.0, 0.205, 0.0), rpy=SHAFT_AXIS_RPY),
            material=polished_steel,
            name="journal_1",
        )
        shaft.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(xyz=(0.0, 0.225, 0.0), rpy=SHAFT_AXIS_RPY),
            material=dark_steel,
            name="retainer_1",
        )
        for name, mesh, y, material in gear_layout[idx]:
            shaft.visual(
                mesh,
                origin=Origin(xyz=(0.0, y, 0.0), rpy=SHAFT_AXIS_RPY),
                material=material,
                name=name,
            )
        model.articulation(
            f"frame_to_shaft_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=shaft,
            origin=Origin(xyz=(x, 0.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("side_frame")
    shafts = [object_model.get_part(f"shaft_{i}") for i in range(4)]
    joints = [object_model.get_articulation(f"frame_to_shaft_{i}") for i in range(4)]

    for idx, joint in enumerate(joints):
        ctx.check(
            f"shaft_{idx}_independent_continuous_axis",
            joint.parent == "side_frame"
            and joint.child == f"shaft_{idx}"
            and joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0)
            and joint.mimic is None,
            details=f"joint={joint}",
        )

    for idx, shaft in enumerate(shafts):
        ctx.allow_overlap(
            frame,
            shaft,
            elem_a=f"front_bearing_{idx}",
            elem_b="journal_0",
            reason="The rotating shaft journal is intentionally captured inside the front bushing sleeve.",
        )
        ctx.allow_overlap(
            frame,
            shaft,
            elem_a=f"rear_bearing_{idx}",
            elem_b="journal_1",
            reason="The rotating shaft journal is intentionally captured inside the rear bushing sleeve.",
        )
        ctx.expect_within(
            shaft,
            frame,
            axes="xz",
            inner_elem="journal_0",
            outer_elem=f"front_bearing_{idx}",
            margin=0.002,
            name=f"shaft_{idx}_front_journal_centered_in_bushing",
        )
        ctx.expect_within(
            shaft,
            frame,
            axes="xz",
            inner_elem="journal_1",
            outer_elem=f"rear_bearing_{idx}",
            margin=0.002,
            name=f"shaft_{idx}_rear_journal_centered_in_bushing",
        )
        ctx.expect_overlap(
            shaft,
            frame,
            axes="xz",
            elem_a="axle",
            elem_b="front_bearing_0" if idx == 0 else f"front_bearing_{idx}",
            min_overlap=0.020,
            name=f"shaft_{idx}_runs_through_front_bearing",
        )
        ctx.expect_overlap(
            shaft,
            frame,
            axes="xz",
            elem_a="axle",
            elem_b=f"rear_bearing_{idx}",
            min_overlap=0.020,
            name=f"shaft_{idx}_runs_through_rear_bearing",
        )

    gear_pairs = (
        (shafts[0], "small_gear", shafts[1], "large_gear"),
        (shafts[1], "small_gear", shafts[2], "large_gear"),
        (shafts[2], "small_gear", shafts[3], "large_gear"),
    )
    for idx, (driver, driver_elem, driven, driven_elem) in enumerate(gear_pairs):
        ctx.expect_overlap(
            driver,
            driven,
            axes="y",
            elem_a=driver_elem,
            elem_b=driven_elem,
            min_overlap=0.020,
            name=f"gear_pair_{idx}_same_face_plane",
        )
        ctx.expect_overlap(
            driver,
            driven,
            axes="xz",
            elem_a=driver_elem,
            elem_b=driven_elem,
            min_overlap=0.006,
            name=f"gear_pair_{idx}_close_in_side_view",
        )

    rest_positions = [ctx.part_world_position(shaft) for shaft in shafts]
    with ctx.pose({joints[0]: 1.2, joints[1]: -0.7, joints[2]: 0.9, joints[3]: -1.1}):
        moved_positions = [ctx.part_world_position(shaft) for shaft in shafts]
    ctx.check(
        "shaft_origins_stay_on_supported_axes",
        rest_positions == moved_positions,
        details=f"rest={rest_positions}, moved={moved_positions}",
    )

    return ctx.report()


object_model = build_object_model()
