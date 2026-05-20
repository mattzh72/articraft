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


def _hinge_barrel_origin(
    *, x: float, y: float, z: float, axis_along_y: bool = True
) -> Origin:
    return Origin(
        xyz=(x, y, z),
        rpy=((math.pi / 2.0, 0.0, 0.0) if axis_along_y else (0.0, 0.0, 0.0)),
    )


def _add_side_stripes(
    part,
    *,
    centers_x: list[float],
    length: float,
    beam_width: float,
    beam_center_z: float,
    stripe_height: float,
    stripe_thickness: float,
    material,
) -> None:
    for center_x in centers_x:
        for side in (-1.0, 1.0):
            part.visual(
                Box((length, stripe_thickness, stripe_height)),
                origin=Origin(
                    xyz=(
                        center_x,
                        side * (beam_width * 0.5 - stripe_thickness * 0.5),
                        beam_center_z,
                    )
                ),
                material=material,
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_road_barrier")

    housing_paint = model.material("housing_paint", rgba=(0.26, 0.29, 0.31, 1.0))
    housing_panel = model.material("housing_panel", rgba=(0.38, 0.41, 0.44, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    boom_white = model.material("boom_white", rgba=(0.94, 0.95, 0.93, 1.0))
    reflector_red = model.material("reflector_red", rgba=(0.75, 0.10, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.06, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.14, 0.28, 0.44)),
        origin=Origin(xyz=(-0.11, 0.0, 0.22)),
        material=housing_paint,
        name="housing_shell",
    )
    housing.visual(
        Box((0.01, 0.32, 0.48)),
        origin=Origin(xyz=(-0.185, 0.0, 0.24)),
        material=steel_dark,
        name="mounting_plate",
    )
    housing.visual(
        Box((0.008, 0.23, 0.11)),
        origin=Origin(xyz=(-0.036, 0.0, 0.345)),
        material=housing_panel,
        name="upper_front_service_door",
    )
    housing.visual(
        Box((0.008, 0.23, 0.11)),
        origin=Origin(xyz=(-0.036, 0.0, 0.095)),
        material=housing_panel,
        name="lower_front_service_door",
    )
    housing.visual(
        Box((0.07, 0.035, 0.12)),
        origin=Origin(xyz=(-0.015, -0.07, 0.22)),
        material=housing_paint,
        name="left_pivot_cheek",
    )
    housing.visual(
        Box((0.07, 0.035, 0.12)),
        origin=Origin(xyz=(-0.015, 0.07, 0.22)),
        material=housing_paint,
        name="right_pivot_cheek",
    )
    housing.visual(
        Cylinder(radius=0.04, length=0.07),
        origin=_hinge_barrel_origin(x=0.0, y=-0.07, z=0.22),
        material=steel_dark,
        name="left_pivot_barrel",
    )
    housing.visual(
        Cylinder(radius=0.04, length=0.07),
        origin=_hinge_barrel_origin(x=0.0, y=0.07, z=0.22),
        material=steel_dark,
        name="right_pivot_barrel",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.20, 0.32, 0.50)),
        mass=32.0,
        origin=Origin(xyz=(-0.095, 0.0, 0.24)),
    )

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        Cylinder(radius=0.038, length=0.07),
        origin=_hinge_barrel_origin(x=0.0, y=0.0, z=0.0),
        material=steel_dark,
        name="root_pivot_barrel",
    )
    inner_arm.visual(
        Box((0.18, 0.06, 0.09)),
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
        material=boom_white,
        name="root_clevis_block",
    )
    inner_arm.visual(
        Box((2.08, 0.10, 0.08)),
        origin=Origin(xyz=(1.22, 0.0, 0.0)),
        material=boom_white,
        name="inner_boom_body",
    )
    inner_arm.visual(
        Box((0.10, 0.10, 0.08)),
        origin=Origin(xyz=(2.17, 0.0, 0.0)),
        material=boom_white,
        name="mid_hinge_transition",
    )
    inner_arm.visual(
        Box((0.12, 0.025, 0.055)),
        origin=Origin(xyz=(2.24, -0.03, 0.015)),
        material=boom_white,
        name="left_mid_hinge_cheek",
    )
    inner_arm.visual(
        Box((0.12, 0.025, 0.055)),
        origin=Origin(xyz=(2.24, 0.03, 0.015)),
        material=boom_white,
        name="right_mid_hinge_cheek",
    )
    inner_arm.visual(
        Cylinder(radius=0.026, length=0.025),
        origin=_hinge_barrel_origin(x=2.30, y=-0.0305, z=0.035),
        material=steel_dark,
        name="left_mid_hinge_barrel",
    )
    inner_arm.visual(
        Cylinder(radius=0.026, length=0.025),
        origin=_hinge_barrel_origin(x=2.30, y=0.0305, z=0.035),
        material=steel_dark,
        name="right_mid_hinge_barrel",
    )
    _add_side_stripes(
        inner_arm,
        centers_x=[0.52, 1.02, 1.52, 2.00],
        length=0.18,
        beam_width=0.10,
        beam_center_z=0.0,
        stripe_height=0.05,
        stripe_thickness=0.004,
        material=reflector_red,
    )
    inner_arm.inertial = Inertial.from_geometry(
        Box((2.35, 0.12, 0.12)),
        mass=18.0,
        origin=Origin(xyz=(1.17, 0.0, 0.01)),
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        Cylinder(radius=0.026, length=0.036),
        origin=_hinge_barrel_origin(x=0.0, y=0.0, z=0.0),
        material=steel_dark,
        name="outer_mid_hinge_barrel",
    )
    outer_arm.visual(
        Box((0.10, 0.034, 0.05)),
        origin=Origin(xyz=(0.05, 0.0, -0.025)),
        material=boom_white,
        name="outer_root_block",
    )
    outer_arm.visual(
        Box((0.18, 0.07, 0.065)),
        origin=Origin(xyz=(0.18, 0.0, -0.0325)),
        material=boom_white,
        name="outer_transition_block",
    )
    outer_arm.visual(
        Box((2.04, 0.095, 0.07)),
        origin=Origin(xyz=(1.28, 0.0, -0.035)),
        material=boom_white,
        name="outer_boom_body",
    )
    outer_arm.visual(
        Box((0.08, 0.11, 0.09)),
        origin=Origin(xyz=(2.34, 0.0, -0.035)),
        material=rubber_black,
        name="tip_bumper",
    )
    _add_side_stripes(
        outer_arm,
        centers_x=[0.52, 1.02, 1.52, 2.00],
        length=0.18,
        beam_width=0.095,
        beam_center_z=-0.035,
        stripe_height=0.046,
        stripe_thickness=0.004,
        material=reflector_red,
    )
    outer_arm.inertial = Inertial.from_geometry(
        Box((2.40, 0.12, 0.10)),
        mass=15.0,
        origin=Origin(xyz=(1.20, 0.0, -0.03)),
    )

    model.articulation(
        "housing_to_inner_arm",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=inner_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.8,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "inner_arm_to_outer_arm",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(2.30, 0.0, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    main_hinge = object_model.get_articulation("housing_to_inner_arm")
    fold_hinge = object_model.get_articulation("inner_arm_to_outer_arm")

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

    ctx.check(
        "main hinge axis is horizontal",
        tuple(main_hinge.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"axis={main_hinge.axis}",
    )
    ctx.check(
        "fold hinge axis is horizontal",
        tuple(fold_hinge.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"axis={fold_hinge.axis}",
    )
    ctx.check(
        "main hinge rotates upward from horizontal",
        (
            main_hinge.motion_limits is not None
            and main_hinge.motion_limits.lower == 0.0
            and main_hinge.motion_limits.upper is not None
            and main_hinge.motion_limits.upper > 1.3
        ),
        details=f"limits={main_hinge.motion_limits}",
    )
    ctx.check(
        "outer hinge folds downward only",
        (
            fold_hinge.motion_limits is not None
            and fold_hinge.motion_limits.lower == 0.0
            and fold_hinge.motion_limits.upper is not None
            and fold_hinge.motion_limits.upper > 1.2
        ),
        details=f"limits={fold_hinge.motion_limits}",
    )

    ctx.expect_contact(housing, inner_arm, name="housing touches inner arm pivot")
    ctx.expect_contact(inner_arm, outer_arm, name="inner and outer arm touch at fold hinge")

    rest_outer_aabb = ctx.part_world_aabb(outer_arm)
    if rest_outer_aabb is None:
        ctx.fail("outer arm has measurable rest AABB", "outer arm AABB unavailable at rest")
    else:
        with ctx.pose({main_hinge: math.radians(78.0)}):
            raised_outer_aabb = ctx.part_world_aabb(outer_arm)
            if raised_outer_aabb is None:
                ctx.fail("outer arm has measurable raised AABB", "outer arm AABB unavailable when raised")
            else:
                ctx.check(
                    "raised boom reaches well above housing",
                    raised_outer_aabb[1][2] > rest_outer_aabb[1][2] + 2.5,
                    details=f"rest_max_z={rest_outer_aabb[1][2]:.3f}, raised_max_z={raised_outer_aabb[1][2]:.3f}",
                )
                ctx.expect_contact(housing, inner_arm, name="housing remains connected to inner arm when raised")

    inner_rest_aabb = ctx.part_world_aabb(inner_arm)
    if inner_rest_aabb is None:
        ctx.fail("inner arm has measurable rest AABB", "inner arm AABB unavailable at rest")
    else:
        with ctx.pose({fold_hinge: math.radians(70.0)}):
            folded_outer_aabb = ctx.part_world_aabb(outer_arm)
            if folded_outer_aabb is None:
                ctx.fail("outer arm has measurable folded AABB", "outer arm AABB unavailable when folded")
            else:
                ctx.check(
                    "outer arm folds distinctly below inner arm",
                    folded_outer_aabb[0][2] < inner_rest_aabb[0][2] - 0.8,
                    details=f"inner_min_z={inner_rest_aabb[0][2]:.3f}, folded_outer_min_z={folded_outer_aabb[0][2]:.3f}",
                )
                ctx.expect_contact(inner_arm, outer_arm, name="fold hinge stays connected when outer arm hangs")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
