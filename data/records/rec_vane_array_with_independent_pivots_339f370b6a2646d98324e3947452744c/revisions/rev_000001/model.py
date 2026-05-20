from __future__ import annotations

from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_shutter_array")

    dark_anodized = Material("dark_anodized_aluminum", rgba=(0.05, 0.055, 0.06, 1.0))
    vane_finish = Material("matte_graphite_vanes", rgba=(0.16, 0.17, 0.18, 1.0))
    worn_edge = Material("slightly_worn_edges", rgba=(0.31, 0.32, 0.32, 1.0))
    pin_steel = Material("brushed_pivot_pins", rgba=(0.72, 0.70, 0.64, 1.0))

    frame = model.part("outer_frame")

    outer_width = 1.18
    outer_height = 0.98
    frame_depth = 0.16
    side_rail = 0.08
    top_rail = 0.08

    frame.visual(
        Box((side_rail, frame_depth, outer_height)),
        origin=Origin(xyz=(-outer_width / 2 + side_rail / 2, 0.0, 0.0)),
        material=dark_anodized,
        name="side_rail_0",
    )
    frame.visual(
        Box((side_rail, frame_depth, outer_height)),
        origin=Origin(xyz=(outer_width / 2 - side_rail / 2, 0.0, 0.0)),
        material=dark_anodized,
        name="side_rail_1",
    )
    frame.visual(
        Box((outer_width, frame_depth, top_rail)),
        origin=Origin(xyz=(0.0, 0.0, outer_height / 2 - top_rail / 2)),
        material=dark_anodized,
        name="top_rail",
    )
    frame.visual(
        Box((outer_width, frame_depth, top_rail)),
        origin=Origin(xyz=(0.0, 0.0, -outer_height / 2 + top_rail / 2)),
        material=dark_anodized,
        name="bottom_rail",
    )
    frame.visual(
        Box((outer_width - 0.06, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, 0.089, outer_height / 2 - 0.105)),
        material=worn_edge,
        name="rear_stiffener_top",
    )
    frame.visual(
        Box((outer_width - 0.06, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, 0.089, -outer_height / 2 + 0.105)),
        material=worn_edge,
        name="rear_stiffener_bottom",
    )

    vane_centers = [-0.36, -0.24, -0.12, 0.0, 0.12, 0.24, 0.36]
    vane_length = 0.86
    vane_depth = 0.110
    vane_thickness = 0.026
    pin_length = 0.052
    pin_radius = 0.012
    pin_center_x = vane_length / 2 + pin_length / 2 - 0.004

    # Fork-like fixed bearing leaves on the inner faces of the side rails make
    # the otherwise simple inspection shutter read as a bank of supported pivots.
    for idx, z in enumerate(vane_centers):
        for side, x_sign in enumerate((-1.0, 1.0)):
            fork_x = x_sign * 0.497
            frame.visual(
                Cylinder(radius=0.018, length=0.035),
                origin=Origin(xyz=(x_sign * 0.4935, 0.0, z), rpy=(0.0, pi / 2, 0.0)),
                material=worn_edge,
                name=f"pivot_bushing_{idx}_{side}",
            )
            for leaf, z_offset in enumerate((-0.026, 0.026)):
                frame.visual(
                    Box((0.028, 0.128, 0.012)),
                    origin=Origin(xyz=(fork_x, 0.0, z + z_offset)),
                    material=worn_edge,
                    name=f"bearing_leaf_{idx}_{side}_{leaf}",
                )

    for idx, z in enumerate(vane_centers):
        vane = model.part(f"vane_{idx}")
        vane.visual(
            Box((vane_length, vane_depth, vane_thickness)),
            origin=Origin(),
            material=vane_finish,
            name="blade",
        )
        # Narrow bright strips along the long edges emphasize the rectangular
        # cross-section and make each rigid vane visually distinct.
        vane.visual(
            Box((vane_length, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, vane_depth / 2 + 0.003, vane_thickness / 2 - 0.003)),
            material=worn_edge,
            name="front_edge",
        )
        vane.visual(
            Box((vane_length, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, -vane_depth / 2 - 0.003, -vane_thickness / 2 + 0.003)),
            material=worn_edge,
            name="rear_edge",
        )
        vane.visual(
            Cylinder(radius=pin_radius, length=pin_length),
            origin=Origin(xyz=(-pin_center_x, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
            material=pin_steel,
            name="pivot_pin_0",
        )
        vane.visual(
            Cylinder(radius=pin_radius, length=pin_length),
            origin=Origin(xyz=(pin_center_x, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
            material=pin_steel,
            name="pivot_pin_1",
        )

        model.articulation(
            f"frame_to_vane_{idx}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=-1.05, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("outer_frame")
    vanes = [object_model.get_part(f"vane_{idx}") for idx in range(7)]
    joints = [object_model.get_articulation(f"frame_to_vane_{idx}") for idx in range(7)]

    ctx.check(
        "seven independent vane joints",
        len(joints) == 7
        and all(joint.parent == "outer_frame" for joint in joints)
        and len({joint.child for joint in joints}) == 7,
        details=f"joints={[joint.name for joint in joints]}",
    )
    ctx.check(
        "all vanes rotate about the long horizontal axis",
        all(joint.axis == (1.0, 0.0, 0.0) for joint in joints)
        and all(joint.motion_limits is not None for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    for idx, vane in enumerate(vanes):
        # The pin is intentionally retained by a coaxial bushing proxy at each
        # side of the fixed frame, while the visible blade remains a separate
        # moving part on its own revolute joint.
        for side in (0, 1):
            ctx.allow_overlap(
                frame,
                vane,
                elem_a=f"pivot_bushing_{idx}_{side}",
                elem_b=f"pivot_pin_{side}",
                reason="The small coaxial pin/bushing insertion represents a captured pivot running through a bearing hole.",
            )
            ctx.expect_within(
                vane,
                frame,
                axes="yz",
                inner_elem=f"pivot_pin_{side}",
                outer_elem=f"pivot_bushing_{idx}_{side}",
                margin=0.001,
                name=f"vane {idx} pin {side} centered in bushing",
            )
            ctx.expect_overlap(
                vane,
                frame,
                axes="x",
                elem_a=f"pivot_pin_{side}",
                elem_b=f"pivot_bushing_{idx}_{side}",
                min_overlap=0.001,
                name=f"vane {idx} pin {side} retained axially",
            )

    def z_size(part_name: str) -> float:
        bounds = ctx.part_world_aabb(object_model.get_part(part_name))
        if bounds is None:
            return 0.0
        lower, upper = bounds
        return float(upper[2] - lower[2])

    rest_vane_0_z = z_size("vane_0")
    rest_vane_1_z = z_size("vane_1")
    with ctx.pose({joints[0]: 0.85}):
        rotated_vane_0_z = z_size("vane_0")
        unchanged_vane_1_z = z_size("vane_1")

    ctx.check(
        "one vane can rotate without driving its neighbor",
        rotated_vane_0_z > rest_vane_0_z + 0.04
        and abs(unchanged_vane_1_z - rest_vane_1_z) < 0.005,
        details=(
            f"vane_0 z: rest={rest_vane_0_z:.3f}, rotated={rotated_vane_0_z:.3f}; "
            f"vane_1 z: rest={rest_vane_1_z:.3f}, posed={unchanged_vane_1_z:.3f}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
