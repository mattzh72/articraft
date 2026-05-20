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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_inspection_hatch")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.08, 0.095, 0.10, 1.0))
    inner_black = model.material("black_epdm_gasket", rgba=(0.005, 0.006, 0.006, 1.0))
    glass = model.material("slightly_tinted_glass", rgba=(0.42, 0.70, 0.85, 0.34))
    steel = model.material("brushed_stainless_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    yellow = model.material("zinc_plated_hardware", rgba=(0.78, 0.66, 0.36, 1.0))
    handle_mat = model.material("satin_black_handle", rgba=(0.015, 0.014, 0.013, 1.0))

    hinge_x = -0.63
    hinge_y = 0.075
    hinge_z = 0.75

    frame = model.part("wall_frame")
    # A heavy architectural/vehicle-style outer frame.  Four members overlap at
    # the corners so the part is one continuous welded/extruded assembly.
    frame.visual(
        Box((1.20, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=anodized,
        name="frame_top",
    )
    frame.visual(
        Box((1.20, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=anodized,
        name="frame_bottom",
    )
    frame.visual(
        Box((0.10, 0.10, 0.82)),
        origin=Origin(xyz=(-0.55, 0.0, 0.75)),
        material=anodized,
        name="frame_side_0",
    )
    frame.visual(
        Box((0.10, 0.10, 0.82)),
        origin=Origin(xyz=(0.55, 0.0, 0.75)),
        material=anodized,
        name="frame_side_1",
    )
    # Proud inner compression seal just behind the hatch leaf.
    frame.visual(
        Box((0.88, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.044, 1.086)),
        material=inner_black,
        name="gasket_top",
    )
    frame.visual(
        Box((0.88, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.044, 0.414)),
        material=inner_black,
        name="gasket_bottom",
    )
    frame.visual(
        Box((0.036, 0.012, 0.64)),
        origin=Origin(xyz=(-0.486, 0.044, 0.75)),
        material=inner_black,
        name="gasket_side_0",
    )
    frame.visual(
        Box((0.036, 0.012, 0.64)),
        origin=Origin(xyz=(0.486, 0.044, 0.75)),
        material=inner_black,
        name="gasket_side_1",
    )
    # Mounting screws tying the frame into the surrounding structure.
    for i, (x, z) in enumerate(((-0.44, 1.15), (0.44, 1.15), (-0.44, 0.35), (0.44, 0.35))):
        frame.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, 0.054, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"frame_screw_{i}",
        )
    # Stationary latch keeper on the jamb, outside the moving sash envelope.
    frame.visual(
        Box((0.050, 0.018, 0.18)),
        origin=Origin(xyz=(0.525, 0.057, 0.75)),
        material=steel,
        name="strike_plate",
    )
    frame.visual(
        Box((0.030, 0.030, 0.060)),
        origin=Origin(xyz=(0.500, 0.078, 0.75)),
        material=yellow,
        name="strike_lug",
    )

    # Three exposed barrel hinges.  Root-side and sash-side knuckles alternate
    # along Z around the same pin axis, leaving real gaps rather than overlaps.
    hinge_stations = (1.02, 0.75, 0.48)
    for station_index, z_world in enumerate(hinge_stations):
        for segment_index, dz in enumerate((-0.055, 0.055)):
            frame.visual(
                Cylinder(radius=0.018, length=0.050),
                origin=Origin(xyz=(hinge_x, hinge_y, z_world + dz)),
                material=steel,
                name=f"fixed_knuckle_{station_index}_{segment_index}",
            )
            frame.visual(
                Box((0.024, 0.050, 0.058)),
                origin=Origin(xyz=(hinge_x + 0.022, 0.040, z_world + dz)),
                material=steel,
                name=f"fixed_leaf_{station_index}_{segment_index}",
            )
            frame.visual(
                Cylinder(radius=0.006, length=0.008),
                origin=Origin(
                    xyz=(hinge_x + 0.036, 0.068, z_world + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=yellow,
                name=f"fixed_leaf_screw_{station_index}_{segment_index}",
            )
        # Visible pin heads at the ends of each hinge unit.
        frame.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(hinge_x, hinge_y, z_world + 0.083)),
            material=yellow,
            name=f"hinge_pin_cap_{station_index}_0",
        )
        frame.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(hinge_x, hinge_y, z_world - 0.083)),
            material=yellow,
            name=f"hinge_pin_cap_{station_index}_1",
        )

    sash = model.part("sash")
    # Child frame is on the hinge line.  In the closed pose the sash extends in
    # local +X and its rear face just kisses the frame gasket/front plane.
    sash.visual(
        Box((0.98, 0.050, 0.070)),
        origin=Origin(xyz=(0.59, 0.0, 0.315)),
        material=anodized,
        name="sash_top_rail",
    )
    sash.visual(
        Box((0.98, 0.050, 0.070)),
        origin=Origin(xyz=(0.59, 0.0, -0.315)),
        material=anodized,
        name="sash_bottom_rail",
    )
    sash.visual(
        Box((0.070, 0.050, 0.700)),
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material=anodized,
        name="left_stile",
    )
    sash.visual(
        Box((0.070, 0.050, 0.700)),
        origin=Origin(xyz=(1.045, 0.0, 0.0)),
        material=anodized,
        name="right_stile",
    )
    # A real transparent infill, held by raised glazing stops instead of being
    # faked as a solid slab.
    sash.visual(
        Box((0.800, 0.020, 0.540)),
        origin=Origin(xyz=(0.59, 0.005, 0.0)),
        material=glass,
        name="glass_panel",
    )
    sash.visual(
        Box((0.840, 0.014, 0.026)),
        origin=Origin(xyz=(0.59, 0.020, 0.287)),
        material=inner_black,
        name="glazing_stop_top",
    )
    sash.visual(
        Box((0.840, 0.014, 0.026)),
        origin=Origin(xyz=(0.59, 0.020, -0.287)),
        material=inner_black,
        name="glazing_stop_bottom",
    )
    sash.visual(
        Box((0.026, 0.014, 0.560)),
        origin=Origin(xyz=(0.178, 0.020, 0.0)),
        material=inner_black,
        name="glazing_stop_side_0",
    )
    sash.visual(
        Box((0.026, 0.014, 0.560)),
        origin=Origin(xyz=(1.002, 0.020, 0.0)),
        material=inner_black,
        name="glazing_stop_side_1",
    )
    # Moving hinge knuckles and their short leaves, connected only in the middle
    # of each hinge where the fixed side leaves a clearance gap.
    for station_index, z_world in enumerate(hinge_stations):
        z_local = z_world - hinge_z
        sash.visual(
            Cylinder(radius=0.018, length=0.052),
            origin=Origin(xyz=(0.0, 0.0, z_local)),
            material=steel,
            name=f"moving_knuckle_{station_index}",
        )
        sash.visual(
            Box((0.120, 0.014, 0.054)),
            origin=Origin(xyz=(0.060, 0.024, z_local)),
            material=steel,
            name=f"moving_leaf_{station_index}",
        )
        sash.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(
                xyz=(0.100, 0.035, z_local),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=yellow,
            name=f"moving_leaf_screw_{station_index}",
        )

    latch = model.part("latch_handle")
    latch.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="pivot_boss",
    )
    latch.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_cap",
    )
    latch.visual(
        Box((0.040, 0.018, 0.230)),
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        material=handle_mat,
        name="grip_bar",
    )
    latch.visual(
        Box((0.060, 0.014, 0.024)),
        origin=Origin(xyz=(-0.048, -0.003, 0.018)),
        material=yellow,
        name="locking_cam",
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.75),
    )
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=latch,
        origin=Origin(xyz=(1.045, 0.038, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-1.2, upper=1.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("wall_frame")
    sash = object_model.get_part("sash")
    latch = object_model.get_part("latch_handle")
    sash_hinge = object_model.get_articulation("frame_to_sash")
    latch_joint = object_model.get_articulation("sash_to_latch")

    with ctx.pose({sash_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            positive_elem="sash_bottom_rail",
            negative_elem="gasket_bottom",
            min_gap=0.0,
            max_gap=0.002,
            name="closed sash seats on frame gasket",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="xz",
            inner_elem="glass_panel",
            margin=0.0,
            name="glazed panel stays within the surrounding frame",
        )
        ctx.expect_contact(
            latch,
            sash,
            elem_a="pivot_boss",
            elem_b="right_stile",
            contact_tol=0.001,
            name="latch boss is mounted to the sash stile",
        )

    closed_right = ctx.part_element_world_aabb(sash, elem="right_stile")
    with ctx.pose({sash_hinge: 1.15, latch_joint: 0.0}):
        open_right = ctx.part_element_world_aabb(sash, elem="right_stile")
    if closed_right is not None and open_right is not None:
        closed_y = (closed_right[0][1] + closed_right[1][1]) / 2.0
        open_y = (open_right[0][1] + open_right[1][1]) / 2.0
        ctx.check(
            "sash opens outward from the hinge",
            open_y > closed_y + 0.70,
            details=f"closed_y={closed_y:.3f}, open_y={open_y:.3f}",
        )
    else:
        ctx.fail("sash opens outward from the hinge", "could not measure right stile aabb")

    return ctx.report()


object_model = build_object_model()
