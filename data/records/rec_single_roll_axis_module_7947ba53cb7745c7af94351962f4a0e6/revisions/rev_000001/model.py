from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


AXIS_Z = 0.24


def _circle_profile(radius: float, segments: int = 72, *, clockwise: bool = False):
    pts = [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]
    return list(reversed(pts)) if clockwise else pts


def _annulus_mesh(outer_radius: float, inner_radius: float, thickness: float, name: str):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, 96),
        [_circle_profile(inner_radius, 96)],
        thickness,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def _end_plate_mesh(name: str):
    geom = ExtrudeWithHolesGeometry(
        # Local profile X becomes world Z after the visual is rotated onto the roll axis.
        rounded_rect_profile(0.42, 0.36, 0.018, corner_segments=8),
        [_circle_profile(0.083, 96)],
        0.050,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_roll_axis_module")

    dark_steel = Material("dark_blasted_steel", rgba=(0.10, 0.11, 0.115, 1.0))
    parkerized = Material("parkerized_black", rgba=(0.015, 0.017, 0.018, 1.0))
    machined = Material("machined_aluminum", rgba=(0.64, 0.66, 0.66, 1.0))
    bearing = Material("bearing_steel", rgba=(0.34, 0.35, 0.36, 1.0))
    blue = Material("blue_index_band", rgba=(0.03, 0.14, 0.42, 1.0))
    brass = Material("etched_index_marks", rgba=(0.86, 0.66, 0.25, 1.0))
    bolt = Material("black_oxide_fasteners", rgba=(0.02, 0.02, 0.022, 1.0))

    support = model.part("support")
    support.visual(
        Box((1.08, 0.42, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="base_plate",
    )
    support.visual(
        Box((1.00, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.195, 0.050)),
        material=parkerized,
        name="front_rail",
    )
    support.visual(
        Box((1.00, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, -0.195, 0.050)),
        material=parkerized,
        name="rear_rail",
    )

    end_top_names = ("end_top_0", "end_top_1")
    end_bottom_names = ("end_bottom_0", "end_bottom_1")
    end_side_pos_names = ("end_side_pos_0", "end_side_pos_1")
    end_side_neg_names = ("end_side_neg_0", "end_side_neg_1")
    outer_bearing_top_names = ("outer_bearing_top_0", "outer_bearing_top_1")
    outer_bearing_bottom_names = ("outer_bearing_bottom_0", "outer_bearing_bottom_1")
    outer_bearing_side_pos_names = ("outer_bearing_side_pos_0", "outer_bearing_side_pos_1")
    outer_bearing_side_neg_names = ("outer_bearing_side_neg_0", "outer_bearing_side_neg_1")
    for i, x in enumerate((-0.40, 0.40)):
        support.visual(
            Box((0.050, 0.360, 0.130)),
            origin=Origin(xyz=(x, 0.0, AXIS_Z + 0.145)),
            material=machined,
            name=end_top_names[i],
        )
        support.visual(
            Box((0.050, 0.360, 0.130)),
            origin=Origin(xyz=(x, 0.0, AXIS_Z - 0.145)),
            material=machined,
            name=end_bottom_names[i],
        )
        support.visual(
            Box((0.050, 0.100, 0.290)),
            origin=Origin(xyz=(x, 0.130, AXIS_Z)),
            material=machined,
            name=end_side_pos_names[i],
        )
        support.visual(
            Box((0.050, 0.100, 0.290)),
            origin=Origin(xyz=(x, -0.130, AXIS_Z)),
            material=machined,
            name=end_side_neg_names[i],
        )
        support.visual(
            Box((0.046, 0.160, 0.026)),
            origin=Origin(xyz=(x, 0.0, AXIS_Z + 0.070)),
            material=bearing,
            name=outer_bearing_top_names[i],
        )
        support.visual(
            Box((0.046, 0.160, 0.026)),
            origin=Origin(xyz=(x, 0.0, AXIS_Z - 0.070)),
            material=bearing,
            name=outer_bearing_bottom_names[i],
        )
        support.visual(
            Box((0.046, 0.026, 0.160)),
            origin=Origin(xyz=(x, 0.070, AXIS_Z)),
            material=bearing,
            name=outer_bearing_side_pos_names[i],
        )
        support.visual(
            Box((0.046, 0.026, 0.160)),
            origin=Origin(xyz=(x, -0.070, AXIS_Z)),
            material=bearing,
            name=outer_bearing_side_neg_names[i],
        )
        support.visual(
            Box((0.060, 0.110, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.047)),
            material=dark_steel,
            name=f"bearing_foot_{i}",
        )
        for j, y in enumerate((-0.172, 0.172)):
            support.visual(
                Box((0.060, 0.018, 0.225)),
                origin=Origin(xyz=(x, y, 0.142)),
                material=dark_steel,
                name=f"side_gusset_{i}_{j}",
            )
        for j, (y, z) in enumerate(
            ((-0.135, AXIS_Z - 0.115), (0.135, AXIS_Z - 0.115), (-0.135, AXIS_Z + 0.115), (0.135, AXIS_Z + 0.115))
        ):
            support.visual(
                Cylinder(0.008, 0.012),
                origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bolt,
                name=f"plate_bolt_{i}_{j}",
            )

    roll_stage = model.part("roll_stage")
    roll_stage.visual(
        Cylinder(0.038, 0.960),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="through_shaft",
    )
    roll_stage.visual(
        _annulus_mesh(0.182, 0.112, 0.052, "central_frame_ring"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="frame_ring",
    )
    # Four machined spokes make the ring visibly structural rather than a decorative disk.
    roll_stage.visual(
        Box((0.060, 0.124, 0.026)),
        origin=Origin(xyz=(0.0, 0.097, 0.0)),
        material=machined,
        name="spoke_y_pos",
    )
    roll_stage.visual(
        Box((0.060, 0.124, 0.026)),
        origin=Origin(xyz=(0.0, -0.097, 0.0)),
        material=machined,
        name="spoke_y_neg",
    )
    roll_stage.visual(
        Box((0.060, 0.026, 0.124)),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=machined,
        name="spoke_z_pos",
    )
    roll_stage.visual(
        Box((0.060, 0.026, 0.124)),
        origin=Origin(xyz=(0.0, 0.0, -0.097)),
        material=machined,
        name="spoke_z_neg",
    )
    for i, x in enumerate((-0.282, 0.282)):
        roll_stage.visual(
            Cylinder(0.070, 0.078),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined,
            name=f"coaxial_hub_{i}",
        )
        roll_stage.visual(
            Cylinder(0.049, 0.026),
            origin=Origin(xyz=(x * 0.90, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=blue,
            name=f"index_band_{i}",
        )
    collar_mesh = _annulus_mesh(0.064, 0.034, 0.022, "split_retaining_collar")
    for i, x in enumerate((-0.40, 0.40)):
        roll_stage.visual(
            Box((0.038, 0.050, 0.020)),
            origin=Origin(xyz=(x, 0.0, 0.047)),
            material=bearing,
            name=f"bearing_race_top_{i}",
        )
        roll_stage.visual(
            Box((0.038, 0.050, 0.020)),
            origin=Origin(xyz=(x, 0.0, -0.047)),
            material=bearing,
            name=f"bearing_race_bottom_{i}",
        )
        roll_stage.visual(
            Box((0.038, 0.020, 0.050)),
            origin=Origin(xyz=(x, 0.047, 0.0)),
            material=bearing,
            name=f"bearing_race_side_pos_{i}",
        )
        roll_stage.visual(
            Box((0.038, 0.020, 0.050)),
            origin=Origin(xyz=(x, -0.047, 0.0)),
            material=bearing,
            name=f"bearing_race_side_neg_{i}",
        )
        inboard_x = x - math.copysign(0.061, x)
        outboard_x = x + math.copysign(0.058, x)
        roll_stage.visual(
            collar_mesh,
            origin=Origin(xyz=(inboard_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=parkerized,
            name=f"inboard_collar_{i}",
        )
        roll_stage.visual(
            collar_mesh,
            origin=Origin(xyz=(outboard_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=parkerized,
            name=f"outboard_collar_{i}",
        )

    # Clamp blocks and exposed socket heads on the ring, including a small index tab.
    roll_stage.visual(
        Box((0.078, 0.104, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=dark_steel,
        name="clamp_block_top",
    )
    roll_stage.visual(
        Box((0.078, 0.104, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.184)),
        material=dark_steel,
        name="clamp_block_bottom",
    )
    roll_stage.visual(
        Box((0.050, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, 0.145, 0.132)),
        material=brass,
        name="index_tab",
    )
    for j, (y, z) in enumerate(((-0.034, 0.184), (0.034, 0.184), (-0.034, -0.184), (0.034, -0.184))):
        roll_stage.visual(
            Cylinder(0.006, 0.088),
            origin=Origin(xyz=(0.0, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bolt,
            name=f"clamp_screw_{j}",
        )

    roll_joint = model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=support,
        child=roll_stage,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.0, lower=-1.75, upper=1.75),
    )

    for i, x in enumerate((-0.429, 0.429)):
        cover = model.part(f"access_cover_{i}")
        outward = -1.0 if x < 0.0 else 1.0
        cover.visual(
            Box((0.008, 0.165, 0.092)),
            origin=Origin(),
            material=parkerized,
            name="cover_plate",
        )
        for j, (y, z) in enumerate(((-0.060, -0.028), (0.060, -0.028), (-0.060, 0.028), (0.060, 0.028))):
            cover.visual(
                Cylinder(0.006, 0.007),
                origin=Origin(xyz=(outward * 0.006, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bolt,
                name=f"cover_screw_{j}",
            )
        model.articulation(
            f"support_to_cover_{i}",
            ArticulationType.FIXED,
            parent=support,
            child=cover,
            origin=Origin(xyz=(x, 0.0, AXIS_Z + 0.105)),
        )

    # A named joint variable is retained for readability and intentional single moving DOF.
    assert roll_joint is not None
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    roll_stage = object_model.get_part("roll_stage")
    roll_axis = object_model.get_articulation("roll_axis")

    limits = roll_axis.motion_limits
    ctx.check(
        "single bounded roll axis",
        roll_axis.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower < -1.0
        and limits.upper > 1.0,
        details=f"axis={roll_axis.axis}, limits={limits}",
    )

    for i in (0, 1):
        ctx.expect_gap(
            support,
            roll_stage,
            axis="z",
            positive_elem=f"outer_bearing_top_{i}",
            negative_elem=f"bearing_race_top_{i}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"upper bearing race {i} is seated",
        )
        ctx.expect_gap(
            roll_stage,
            support,
            axis="z",
            positive_elem=f"bearing_race_bottom_{i}",
            negative_elem=f"outer_bearing_bottom_{i}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"lower bearing race {i} is seated",
        )
        ctx.expect_gap(
            support,
            roll_stage,
            axis="y",
            positive_elem=f"outer_bearing_side_pos_{i}",
            negative_elem=f"bearing_race_side_pos_{i}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"positive side bearing race {i} is seated",
        )
        ctx.expect_gap(
            roll_stage,
            support,
            axis="y",
            positive_elem=f"bearing_race_side_neg_{i}",
            negative_elem=f"outer_bearing_side_neg_{i}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"negative side bearing race {i} is seated",
        )

    ctx.expect_contact(
        "access_cover_0",
        support,
        elem_a="cover_plate",
        elem_b="end_top_0",
        contact_tol=0.0005,
        name="negative access cover mounts to end support",
    )
    ctx.expect_contact(
        "access_cover_1",
        support,
        elem_a="cover_plate",
        elem_b="end_top_1",
        contact_tol=0.0005,
        name="positive access cover mounts to end support",
    )

    def _center_z_y(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return ((mn[2] + mx[2]) * 0.5, (mn[1] + mx[1]) * 0.5)

    rest = _center_z_y(ctx.part_element_world_aabb(roll_stage, elem="clamp_block_top"))
    with ctx.pose({roll_axis: 0.75}):
        moved = _center_z_y(ctx.part_element_world_aabb(roll_stage, elem="clamp_block_top"))
    ctx.check(
        "clamp block follows roll motion",
        rest is not None
        and moved is not None
        and moved[0] < rest[0] - 0.02
        and moved[1] < rest[1] - 0.08,
        details=f"rest_zy={rest}, moved_zy={moved}",
    )

    return ctx.report()


object_model = build_object_model()
