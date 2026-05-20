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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_W = 0.220
OUTER_D = 0.170
BASE_H = 0.100
WALL = 0.005
FLOOR = 0.008
HINGE_AXIS_Y = -0.083
HINGE_AXIS_Z = 0.132


def _center_z(aabb):
    return (aabb[0][2] + aabb[1][2]) * 0.5


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_box")

    body_shell = model.material("body_shell", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_trim = model.material("satin_trim", rgba=(0.24, 0.24, 0.26, 1.0))
    soft_pad = model.material("soft_pad", rgba=(0.44, 0.42, 0.38, 1.0))
    dark_mechanism = model.material("dark_mechanism", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base_shell")
    base.visual(
        Box((OUTER_W, OUTER_D, FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR * 0.5)),
        material=body_shell,
        name="floor_pan",
    )
    base.visual(
        Box((WALL, OUTER_D, BASE_H - 0.004)),
        origin=Origin(xyz=(-(OUTER_W * 0.5) + WALL * 0.5, 0.0, (BASE_H - 0.004) * 0.5)),
        material=body_shell,
        name="left_wall",
    )
    base.visual(
        Box((WALL, OUTER_D, BASE_H - 0.004)),
        origin=Origin(xyz=((OUTER_W * 0.5) - WALL * 0.5, 0.0, (BASE_H - 0.004) * 0.5)),
        material=body_shell,
        name="right_wall",
    )
    base.visual(
        Box((OUTER_W - 0.008, WALL, BASE_H - 0.004)),
        origin=Origin(xyz=(0.0, (OUTER_D * 0.5) - WALL * 0.5, (BASE_H - 0.004) * 0.5)),
        material=body_shell,
        name="front_wall",
    )
    base.visual(
        Box((OUTER_W - 0.008, WALL, BASE_H - 0.004)),
        origin=Origin(xyz=(0.0, -(OUTER_D * 0.5) + WALL * 0.5, (BASE_H - 0.004) * 0.5)),
        material=body_shell,
        name="back_wall",
    )
    base.visual(
        Box((0.020, OUTER_D - 0.010, 0.012)),
        origin=Origin(xyz=(-(OUTER_W * 0.5) + 0.015, 0.0, 0.014)),
        material=satin_trim,
        name="left_inner_rib",
    )
    base.visual(
        Box((0.020, OUTER_D - 0.010, 0.012)),
        origin=Origin(xyz=((OUTER_W * 0.5) - 0.015, 0.0, 0.014)),
        material=satin_trim,
        name="right_inner_rib",
    )
    base.visual(
        Box((0.008, OUTER_D - 0.022, 0.004)),
        origin=Origin(xyz=(-0.104, 0.0, 0.094)),
        material=satin_trim,
        name="left_seat_rail",
    )
    base.visual(
        Box((0.008, OUTER_D - 0.022, 0.004)),
        origin=Origin(xyz=(0.104, 0.0, 0.094)),
        material=satin_trim,
        name="right_seat_rail",
    )
    base.visual(
        Box((OUTER_W - 0.026, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.078, 0.094)),
        material=satin_trim,
        name="front_seat_rail",
    )
    base.visual(
        Box((0.040, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.073, 0.090)),
        material=dark_mechanism,
        name="front_snap_receiver",
    )
    base.visual(
        Box((0.052, 0.055, 0.050)),
        origin=Origin(xyz=(-0.079, 0.0, 0.033)),
        material=body_shell,
        name="left_support_tower",
    )
    base.visual(
        Box((0.052, 0.055, 0.050)),
        origin=Origin(xyz=(0.079, 0.0, 0.033)),
        material=body_shell,
        name="right_support_tower",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(-0.049, 0.0, 0.058), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_mechanism,
        name="left_bearing_pad",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.049, 0.0, 0.058), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_mechanism,
        name="right_bearing_pad",
    )
    base.visual(
        Box((0.020, 0.060, 0.028)),
        origin=Origin(xyz=(0.119, 0.0, 0.038)),
        material=dark_mechanism,
        name="motor_pod",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.044),
        origin=Origin(xyz=(-0.082, HINGE_AXIS_Y, HINGE_AXIS_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_mechanism,
        name="left_hinge_barrel",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.044),
        origin=Origin(xyz=(0.082, HINGE_AXIS_Y, HINGE_AXIS_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_mechanism,
        name="right_hinge_barrel",
    )
    base.visual(
        Box((0.044, 0.008, 0.036)),
        origin=Origin(xyz=(-0.082, HINGE_AXIS_Y, 0.114)),
        material=dark_mechanism,
        name="left_hinge_mount",
    )
    base.visual(
        Box((0.044, 0.008, 0.036)),
        origin=Origin(xyz=(0.082, HINGE_AXIS_Y, 0.114)),
        material=dark_mechanism,
        name="right_hinge_mount",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.240, OUTER_D, 0.145)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((OUTER_W, OUTER_D - 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.090, -0.004)),
        material=body_shell,
        name="top_panel",
    )
    lid.visual(
        Box((WALL, OUTER_D - 0.006, 0.032)),
        origin=Origin(xyz=(-(OUTER_W * 0.5) + WALL * 0.5, 0.089, -0.016)),
        material=body_shell,
        name="left_outer_frame",
    )
    lid.visual(
        Box((WALL, OUTER_D - 0.006, 0.032)),
        origin=Origin(xyz=((OUTER_W * 0.5) - WALL * 0.5, 0.089, -0.016)),
        material=body_shell,
        name="right_outer_frame",
    )
    lid.visual(
        Box((OUTER_W - 0.010, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, 0.170, -0.016)),
        material=body_shell,
        name="front_outer_frame",
    )
    lid.visual(
        Box((0.003, OUTER_D - 0.018, 0.036)),
        origin=Origin(xyz=(-0.1005, 0.083, -0.018)),
        material=satin_trim,
        name="left_inner_lip",
    )
    lid.visual(
        Box((0.003, OUTER_D - 0.018, 0.036)),
        origin=Origin(xyz=(0.1005, 0.083, -0.018)),
        material=satin_trim,
        name="right_inner_lip",
    )
    lid.visual(
        Box((OUTER_W - 0.032, 0.003, 0.036)),
        origin=Origin(xyz=(0.0, 0.159, -0.018)),
        material=satin_trim,
        name="front_inner_lip",
    )
    lid.visual(
        Box((0.118, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.004, -0.012)),
        material=dark_mechanism,
        name="hinge_spine",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_mechanism,
        name="center_hinge_barrel",
    )
    lid.visual(
        Box((0.028, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.163, -0.024)),
        material=dark_mechanism,
        name="snap_tongue",
    )
    lid.inertial = Inertial.from_geometry(
        Box((OUTER_W, OUTER_D, 0.040)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.083, -0.016)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.032, length=0.062),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_mechanism,
        name="drum_body",
    )
    cradle.visual(
        Cylinder(radius=0.036, length=0.004),
        origin=Origin(xyz=(-0.033, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_trim,
        name="left_end_flange",
    )
    cradle.visual(
        Cylinder(radius=0.036, length=0.004),
        origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_trim,
        name="right_end_flange",
    )
    cradle.visual(
        Box((0.064, 0.036, 0.044)),
        origin=Origin(xyz=(0.0, 0.022, 0.000)),
        material=soft_pad,
        name="watch_pillow",
    )
    cradle.visual(
        Box((0.072, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.037, 0.0)),
        material=soft_pad,
        name="pillow_cap",
    )
    cradle.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_mechanism,
        name="left_trunnion",
    )
    cradle.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_mechanism,
        name="right_trunnion",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.090, 0.080, 0.080)),
        mass=0.22,
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.28,
        ),
    )

    model.articulation(
        "base_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_shell")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("base_to_lid")
    cradle_spin = object_model.get_articulation("base_to_cradle")

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

    with ctx.pose({lid_hinge: 0.0, cradle_spin: 0.0}):
        ctx.expect_contact(
            cradle,
            base,
            elem_a="left_trunnion",
            elem_b="left_bearing_pad",
            contact_tol=5e-4,
            name="left trunnion is supported by left bearing pad",
        )
        ctx.expect_contact(
            cradle,
            base,
            elem_a="right_trunnion",
            elem_b="right_bearing_pad",
            contact_tol=5e-4,
            name="right trunnion is supported by right bearing pad",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="front_inner_lip",
            negative_elem="front_seat_rail",
            max_gap=8e-4,
            max_penetration=0.0,
            name="front lid lip seats on front rail",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="left_inner_lip",
            negative_elem="left_seat_rail",
            max_gap=8e-4,
            max_penetration=0.0,
            name="left lid lip seats on left rail",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="right_inner_lip",
            negative_elem="right_seat_rail",
            max_gap=8e-4,
            max_penetration=0.0,
            name="right lid lip seats on right rail",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.14,
            name="closed lid covers the presentation box opening",
        )
        ctx.expect_within(
            cradle,
            base,
            axes="xy",
            margin=0.0,
            name="rotating cradle stays within base footprint",
        )

    with ctx.pose({cradle_spin: math.pi * 0.5, lid_hinge: 0.0}):
        ctx.expect_contact(
            cradle,
            base,
            elem_a="left_trunnion",
            elem_b="left_bearing_pad",
            contact_tol=5e-4,
            name="left trunnion remains supported while cradle spins",
        )
        ctx.expect_contact(
            cradle,
            base,
            elem_a="right_trunnion",
            elem_b="right_bearing_pad",
            contact_tol=5e-4,
            name="right trunnion remains supported while cradle spins",
        )

    with ctx.pose({lid_hinge: 0.0}):
        closed_front = ctx.part_element_world_aabb(lid, elem="front_outer_frame")
    with ctx.pose({lid_hinge: 1.10}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_outer_frame")
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="front_outer_frame",
            negative_elem="front_wall",
            min_gap=0.030,
            name="opened lid front clears the box front wall",
        )
    ctx.check(
        "lid opens upward around rear hinge",
        closed_front is not None
        and open_front is not None
        and _center_z(open_front) > _center_z(closed_front) + 0.055,
        details=(
            f"closed_front={closed_front}, open_front={open_front}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
