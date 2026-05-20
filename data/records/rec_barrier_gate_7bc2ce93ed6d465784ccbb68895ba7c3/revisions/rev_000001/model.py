from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pivot_drop_arm_parking_barrier")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.66, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.93, 0.78, 0.12, 1.0))
    arm_white = model.material("arm_white", rgba=(0.94, 0.95, 0.95, 1.0))
    reflective_red = model.material("reflective_red", rgba=(0.76, 0.12, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    curb = model.part("curb")
    curb.visual(
        Box((3.50, 0.36, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=concrete,
        name="curb_body",
    )
    curb.inertial = Inertial.from_geometry(
        Box((3.50, 0.36, 0.16)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    main_post = model.part("main_post")
    main_post.visual(
        Box((0.26, 0.26, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="base_plate",
    )
    main_post.visual(
        Box((0.16, 0.16, 0.76)),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=safety_yellow,
        name="shaft",
    )
    main_post.visual(
        Box((0.20, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=dark_steel,
        name="top_cap",
    )
    main_post.visual(
        Cylinder(radius=0.075, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        material=dark_steel,
        name="pivot_pad",
    )
    main_post.visual(
        Box((0.08, 0.06, 0.06)),
        origin=Origin(xyz=(-0.10, 0.09, 0.80)),
        material=dark_steel,
        name="stop_block",
    )
    main_post.inertial = Inertial.from_geometry(
        Box((0.26, 0.26, 0.83)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
    )

    receiving_post = model.part("receiving_post")
    receiving_post.visual(
        Box((0.22, 0.22, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="base_plate",
    )
    receiving_post.visual(
        Box((0.12, 0.12, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=safety_yellow,
        name="shaft",
    )
    receiving_post.visual(
        Box((0.15, 0.15, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=dark_steel,
        name="top_cap",
    )
    receiving_post.visual(
        Box((0.035, 0.09, 0.14)),
        origin=Origin(xyz=(-0.0775, 0.0, 0.69)),
        material=dark_steel,
        name="socket_back",
    )
    receiving_post.visual(
        Box((0.10, 0.09, 0.02)),
        origin=Origin(xyz=(-0.11, 0.0, 0.755)),
        material=dark_steel,
        name="socket_roof",
    )
    receiving_post.visual(
        Box((0.10, 0.012, 0.09)),
        origin=Origin(xyz=(-0.11, 0.039, 0.705)),
        material=dark_steel,
        name="left_cheek",
    )
    receiving_post.visual(
        Box((0.10, 0.012, 0.09)),
        origin=Origin(xyz=(-0.11, -0.039, 0.705)),
        material=dark_steel,
        name="right_cheek",
    )
    receiving_post.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.78)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    arm = model.part("barrier_arm")
    arm.visual(
        Cylinder(radius=0.07, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="pivot_collar",
    )
    arm.visual(
        Box((0.22, 0.06, 0.10)),
        origin=Origin(xyz=(0.10, 0.0, 0.055)),
        material=arm_white,
        name="pivot_head",
    )
    arm.visual(
        Box((2.80, 0.05, 0.08)),
        origin=Origin(xyz=(1.40, 0.0, 0.06)),
        material=arm_white,
        name="beam_shell",
    )
    arm.visual(
        Box((0.48, 0.052, 0.016)),
        origin=Origin(xyz=(0.56, 0.0, 0.096)),
        material=reflective_red,
        name="stripe_a",
    )
    arm.visual(
        Box((0.48, 0.052, 0.016)),
        origin=Origin(xyz=(1.42, 0.0, 0.096)),
        material=reflective_red,
        name="stripe_b",
    )
    arm.visual(
        Box((0.48, 0.052, 0.016)),
        origin=Origin(xyz=(2.28, 0.0, 0.096)),
        material=reflective_red,
        name="stripe_c",
    )
    arm.visual(
        Box((0.12, 0.008, 0.090)),
        origin=Origin(xyz=(2.855, 0.019, -0.005)),
        material=dark_steel,
        name="left_clevis",
    )
    arm.visual(
        Box((0.12, 0.008, 0.090)),
        origin=Origin(xyz=(2.855, -0.019, -0.005)),
        material=dark_steel,
        name="right_clevis",
    )
    arm.inertial = Inertial.from_geometry(
        Box((3.07, 0.10, 0.12)),
        mass=18.0,
        origin=Origin(xyz=(1.425, 0.0, 0.03)),
    )

    latch = model.part("latch")
    latch.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    latch.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    latch.visual(
        Box((0.020, 0.024, 0.024)),
        origin=Origin(xyz=(0.006, 0.0, -0.006)),
        material=dark_steel,
        name="hanger_block",
    )
    latch.visual(
        Box((0.022, 0.012, 0.090)),
        origin=Origin(xyz=(0.012, 0.0, -0.053)),
        material=dark_steel,
        name="hasp_body",
    )
    latch.visual(
        Box((0.078, 0.012, 0.016)),
        origin=Origin(xyz=(0.051, 0.0, -0.095)),
        material=dark_steel,
        name="lower_bridge",
    )
    latch.visual(
        Box((0.026, 0.012, 0.022)),
        origin=Origin(xyz=(0.081, 0.0, -0.097)),
        material=dark_steel,
        name="hook_nose",
    )
    latch.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.050, 0.0, -0.095), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lock_eye",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.06, 0.03, 0.11)),
        mass=0.8,
        origin=Origin(xyz=(0.03, 0.0, -0.045)),
    )

    model.articulation(
        "curb_to_main_post",
        ArticulationType.FIXED,
        parent=curb,
        child=main_post,
        origin=Origin(xyz=(-1.48, 0.0, 0.16)),
    )
    model.articulation(
        "curb_to_receiving_post",
        ArticulationType.FIXED,
        parent=curb,
        child=receiving_post,
        origin=Origin(xyz=(1.60, 0.0, 0.16)),
    )
    model.articulation(
        "barrier_swing",
        ArticulationType.REVOLUTE,
        parent=main_post,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.83)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "latch_hinge",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=latch,
        origin=Origin(xyz=(2.85, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((a + b) * 0.5 for a, b in zip(lo, hi))

    ctx = TestContext(object_model)
    curb = object_model.get_part("curb")
    main_post = object_model.get_part("main_post")
    receiving_post = object_model.get_part("receiving_post")
    arm = object_model.get_part("barrier_arm")
    latch = object_model.get_part("latch")
    barrier_swing = object_model.get_articulation("barrier_swing")
    latch_hinge = object_model.get_articulation("latch_hinge")

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

    ctx.expect_contact(
        main_post,
        curb,
        elem_a="base_plate",
        elem_b="curb_body",
        name="main post base plate seats on curb",
    )
    ctx.expect_contact(
        receiving_post,
        curb,
        elem_a="base_plate",
        elem_b="curb_body",
        name="receiving post base plate seats on curb",
    )

    closed_tip_center = None
    open_tip_center = None

    with ctx.pose({barrier_swing: 0.0, latch_hinge: 0.0}):
        ctx.expect_contact(
            arm,
            main_post,
            elem_a="pivot_collar",
            elem_b="pivot_pad",
            name="arm collar rests on main pivot pad",
        )
        ctx.expect_contact(
            latch,
            arm,
            elem_a="left_trunnion",
            elem_b="left_clevis",
            name="left trunnion bears on left clevis plate",
        )
        ctx.expect_contact(
            latch,
            arm,
            elem_a="right_trunnion",
            elem_b="right_clevis",
            name="right trunnion bears on right clevis plate",
        )
        ctx.expect_gap(
            arm,
            receiving_post,
            axis="z",
            positive_elem="beam_shell",
            negative_elem="top_cap",
            min_gap=0.03,
            name="closed arm clears receiving post top",
        )
        ctx.expect_gap(
            receiving_post,
            latch,
            axis="z",
            positive_elem="socket_roof",
            negative_elem="hook_nose",
            min_gap=0.0,
            max_gap=0.012,
            name="closed latch hook sits just below socket roof",
        )
        ctx.expect_overlap(
            latch,
            receiving_post,
            axes="xy",
            elem_a="hook_nose",
            elem_b="socket_roof",
            min_overlap=0.010,
            name="closed latch hook aligns with receiver socket",
        )
        closed_tip_center = _center(ctx.part_element_world_aabb(arm, elem="left_clevis"))

    with ctx.pose({barrier_swing: 0.0, latch_hinge: 1.15}):
        ctx.expect_gap(
            latch,
            receiving_post,
            axis="z",
            positive_elem="hook_nose",
            negative_elem="socket_roof",
            min_gap=0.012,
            name="opened latch lifts clear of receiver socket",
        )

    with ctx.pose({barrier_swing: 1.35, latch_hinge: 1.15}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open pose has no part overlap")
        open_tip_center = _center(ctx.part_element_world_aabb(arm, elem="left_clevis"))

    ctx.check(
        "arm tip swings outboard when opened",
        closed_tip_center is not None
        and open_tip_center is not None
        and open_tip_center[1] > closed_tip_center[1] + 2.0,
        details=f"closed_tip={closed_tip_center}, open_tip={open_tip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
