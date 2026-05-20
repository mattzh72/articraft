from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="independent_louver_shutter")

    satin_aluminum = model.material("satin_aluminum", color=(0.74, 0.76, 0.74, 1.0))
    dark_bushing = model.material("dark_bushing", color=(0.08, 0.085, 0.08, 1.0))
    pale_blade = model.material("powder_coated_slats", color=(0.86, 0.88, 0.84, 1.0))
    pin_steel = model.material("brushed_pin_steel", color=(0.58, 0.60, 0.60, 1.0))
    black_grip = model.material("black_thumb_tabs", color=(0.03, 0.03, 0.028, 1.0))

    frame = model.part("frame")

    # A rigid cage: paired front/rear side stiles and rails leave the pivot
    # centers visually open, while the small bearing rings carry each slat pin.
    frame_width = 0.96
    frame_height = 1.08
    side_x = 0.475
    rail_y = 0.060
    stile_size = (0.035, 0.025, 1.12)
    horizontal_rail = (frame_width, 0.025, 0.045)
    side_rail = (0.035, 0.140, 0.045)

    for x_sign, x_label in ((-1.0, "left"), (1.0, "right")):
        x = x_sign * side_x
        for y_sign, y_label in ((-1.0, "front"), (1.0, "rear")):
            y = y_sign * rail_y
            frame.visual(
                Box(stile_size),
                origin=Origin(xyz=(x, y, 0.0)),
                material=satin_aluminum,
                name=f"{x_label}_{y_label}_stile",
            )
        for z_sign, z_label in ((-1.0, "lower"), (1.0, "upper")):
            z = z_sign * frame_height / 2.0
            frame.visual(
                Box(side_rail),
                origin=Origin(xyz=(x, 0.0, z)),
                material=satin_aluminum,
                name=f"{x_label}_{z_label}_side_rail",
            )

    for y_sign, y_label in ((-1.0, "front"), (1.0, "rear")):
        y = y_sign * rail_y
        for z_sign, z_label in ((-1.0, "lower"), (1.0, "upper")):
            z = z_sign * frame_height / 2.0
            frame.visual(
                Box(horizontal_rail),
                origin=Origin(xyz=(0.0, y, z)),
                material=satin_aluminum,
                name=f"{y_label}_{z_label}_rail",
            )

    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.0125, tube=0.0050, radial_segments=20, tubular_segments=12),
        "pivot_bearing_ring",
    )

    slat_count = 8
    slat_pitch = 0.105
    bottom_z = -0.3675
    slat_zs = [bottom_z + i * slat_pitch for i in range(slat_count)]

    for i, z in enumerate(slat_zs):
        for x_sign, side_name in ((-1.0, "left"), (1.0, "right")):
            x = x_sign * 0.455
            frame.visual(
                bearing_mesh,
                origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_bushing,
                name=f"{side_name}_bearing_{i}",
            )

            # Two small carrier webs tie each bearing ring into the front/rear
            # side stiles without blocking the pin bore.
            for y_sign, y_label in ((-1.0, "front"), (1.0, "rear")):
                frame.visual(
                    Box((0.016, 0.034, 0.007)),
                    origin=Origin(xyz=(x, y_sign * 0.0325, z)),
                    material=satin_aluminum,
                    name=f"{side_name}_{y_label}_web_{i}",
                )

        slat = model.part(f"slat_{i}")
        # The blade itself is narrow and rounded at both long edges.  Its
        # separate steel pins sit on the joint axis and pass through the bearing
        # rings; a small right-side thumb tab shows that each slat can be set
        # independently.
        slat.visual(
            Box((0.780, 0.080, 0.010)),
            origin=Origin(),
            material=pale_blade,
            name="blade_web",
        )
        for y_sign, edge_name in ((-1.0, "front_edge"), (1.0, "rear_edge")):
            slat.visual(
                Cylinder(radius=0.0055, length=0.780),
                origin=Origin(xyz=(0.0, y_sign * 0.040, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=pale_blade,
                name=edge_name,
            )
        slat.visual(
            Cylinder(radius=0.0078, length=0.082),
            origin=Origin(xyz=(-0.427, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pin_steel,
            name="left_pin",
        )
        slat.visual(
            Cylinder(radius=0.0078, length=0.106),
            origin=Origin(xyz=(0.439, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pin_steel,
            name="right_pin",
        )
        slat.visual(
            Box((0.012, 0.018, 0.038)),
            origin=Origin(xyz=(0.491, 0.0, 0.024)),
            material=black_grip,
            name="thumb_tab",
        )

        model.articulation(
            f"frame_to_slat_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            # Zero pose is slightly down-tilted like a half-open louver stack.
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(-0.35, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=-0.45, upper=0.75),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    slats = [object_model.get_part(f"slat_{i}") for i in range(8)]
    joints = [object_model.get_articulation(f"frame_to_slat_{i}") for i in range(8)]

    ctx.check("eight independent slats", len(slats) == 8 and len(joints) == 8)
    for i, joint in enumerate(joints):
        limits = joint.motion_limits
        ctx.allow_overlap(
            slats[i],
            frame,
            elem_a="left_pin",
            elem_b=f"left_bearing_{i}",
            reason="The slat pivot pin is intentionally captured in the bearing ring with a slight interference fit.",
        )
        ctx.allow_overlap(
            slats[i],
            frame,
            elem_a="right_pin",
            elem_b=f"right_bearing_{i}",
            reason="The slat pivot pin is intentionally captured in the bearing ring with a slight interference fit.",
        )
        ctx.check(
            f"slat_{i} independent x pivot",
            joint.parent == "frame"
            and joint.child == f"slat_{i}"
            and tuple(joint.axis) == (1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
        )
        ctx.expect_within(
            slats[i],
            frame,
            axes="yz",
            inner_elem="left_pin",
            outer_elem=f"left_bearing_{i}",
            margin=0.0,
            name=f"left pin centered in bearing {i}",
        )
        ctx.expect_overlap(
            slats[i],
            frame,
            axes="x",
            elem_a="left_pin",
            elem_b=f"left_bearing_{i}",
            min_overlap=0.006,
            name=f"left pin retained in bearing {i}",
        )
        ctx.expect_within(
            slats[i],
            frame,
            axes="yz",
            inner_elem="right_pin",
            outer_elem=f"right_bearing_{i}",
            margin=0.0,
            name=f"right pin centered in bearing {i}",
        )
        ctx.expect_overlap(
            slats[i],
            frame,
            axes="x",
            elem_a="right_pin",
            elem_b=f"right_bearing_{i}",
            min_overlap=0.006,
            name=f"right pin retained in bearing {i}",
        )

    # A decisive pose check: moving one slat must rotate it without dragging its
    # neighbors, because the prompt calls for independent pivots.
    tracked = slats[3]
    neighbor = slats[4]
    before = ctx.part_element_world_aabb(tracked, elem="blade_web")
    neighbor_before = ctx.part_element_world_aabb(neighbor, elem="blade_web")
    with ctx.pose({joints[3]: -0.45}):
        after = ctx.part_element_world_aabb(tracked, elem="blade_web")
        neighbor_after = ctx.part_element_world_aabb(neighbor, elem="blade_web")
    ctx.check(
        "one slat changes silhouette",
        before is not None
        and after is not None
        and abs((after[1][2] - after[0][2]) - (before[1][2] - before[0][2])) > 0.015,
    )
    ctx.check(
        "neighbor remains independent",
        neighbor_before is not None and neighbor_after is not None and neighbor_before == neighbor_after,
    )

    return ctx.report()


object_model = build_object_model()
