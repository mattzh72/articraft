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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_box_fan")

    body_white = model.material("body_white", rgba=(0.92, 0.93, 0.95, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.65, 0.67, 0.70, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.18, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.195, 0.015)),
        material=trim_gray,
        name="right_foot",
    )
    base.visual(
        Box((0.18, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, -0.195, 0.015)),
        material=trim_gray,
        name="left_foot",
    )
    base.visual(
        Box((0.05, 0.39, 0.032)),
        origin=Origin(xyz=(-0.078, 0.0, 0.016)),
        material=trim_gray,
        name="rear_crossbar",
    )
    base.visual(
        Box((0.032, 0.022, 0.192)),
        origin=Origin(xyz=(-0.076, 0.204, 0.126)),
        material=trim_gray,
        name="right_upright",
    )
    base.visual(
        Box((0.032, 0.022, 0.192)),
        origin=Origin(xyz=(-0.076, -0.204, 0.126)),
        material=trim_gray,
        name="left_upright",
    )
    base.visual(
        Box((0.082, 0.022, 0.03)),
        origin=Origin(xyz=(-0.033, 0.206, 0.204)),
        material=trim_gray,
        name="right_pivot_arm",
    )
    base.visual(
        Box((0.082, 0.022, 0.03)),
        origin=Origin(xyz=(-0.033, -0.206, 0.204)),
        material=trim_gray,
        name="left_pivot_arm",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.0, 0.206, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_pivot_ear",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.0, -0.206, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_pivot_ear",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.36),
        origin=Origin(xyz=(-0.085, 0.0, 0.044), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="base_tube",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.22, 0.45, 0.24)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    housing = model.part("housing")
    outer = 0.34
    frame = 0.024
    depth = 0.10
    bezel_depth = 0.016
    shell_depth = depth - 2.0 * bezel_depth
    opening = outer - 2.0 * frame
    grille_thickness = 0.004
    bezel_front_x = depth * 0.5 - bezel_depth * 0.5
    bezel_rear_x = -depth * 0.5 + bezel_depth * 0.5
    grille_front_x = bezel_front_x - 0.004
    grille_rear_x = bezel_rear_x + 0.004

    housing.visual(
        Box((shell_depth, frame, outer)),
        origin=Origin(xyz=(0.0, outer * 0.5 - frame * 0.5, 0.0)),
        material=body_white,
        name="right_side_shell",
    )
    housing.visual(
        Box((shell_depth, frame, outer)),
        origin=Origin(xyz=(0.0, -outer * 0.5 + frame * 0.5, 0.0)),
        material=body_white,
        name="left_side_shell",
    )
    housing.visual(
        Box((shell_depth, opening, frame)),
        origin=Origin(xyz=(0.0, 0.0, outer * 0.5 - frame * 0.5)),
        material=body_white,
        name="top_shell",
    )
    housing.visual(
        Box((shell_depth, opening, frame)),
        origin=Origin(xyz=(0.0, 0.0, -outer * 0.5 + frame * 0.5)),
        material=body_white,
        name="bottom_shell",
    )
    housing.visual(
        Box((bezel_depth, frame, outer)),
        origin=Origin(xyz=(bezel_front_x, outer * 0.5 - frame * 0.5, 0.0)),
        material=body_white,
        name="front_right_frame",
    )
    housing.visual(
        Box((bezel_depth, frame, outer)),
        origin=Origin(xyz=(bezel_front_x, -outer * 0.5 + frame * 0.5, 0.0)),
        material=body_white,
        name="front_left_frame",
    )
    housing.visual(
        Box((bezel_depth, opening, frame)),
        origin=Origin(xyz=(bezel_front_x, 0.0, outer * 0.5 - frame * 0.5)),
        material=body_white,
        name="front_top_frame",
    )
    housing.visual(
        Box((bezel_depth, opening, frame)),
        origin=Origin(xyz=(bezel_front_x, 0.0, -outer * 0.5 + frame * 0.5)),
        material=body_white,
        name="front_bottom_frame",
    )
    housing.visual(
        Box((bezel_depth, frame, outer)),
        origin=Origin(xyz=(bezel_rear_x, outer * 0.5 - frame * 0.5, 0.0)),
        material=body_white,
        name="rear_right_frame",
    )
    housing.visual(
        Box((bezel_depth, frame, outer)),
        origin=Origin(xyz=(bezel_rear_x, -outer * 0.5 + frame * 0.5, 0.0)),
        material=body_white,
        name="rear_left_frame",
    )
    housing.visual(
        Box((bezel_depth, opening, frame)),
        origin=Origin(xyz=(bezel_rear_x, 0.0, outer * 0.5 - frame * 0.5)),
        material=body_white,
        name="rear_top_frame",
    )
    housing.visual(
        Box((bezel_depth, opening, frame)),
        origin=Origin(xyz=(bezel_rear_x, 0.0, -outer * 0.5 + frame * 0.5)),
        material=body_white,
        name="rear_bottom_frame",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.0, 0.181, -0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_bushing",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.0, -0.181, -0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_side_cap",
    )
    housing.visual(
        Cylinder(radius=0.023, length=0.028),
        origin=Origin(xyz=(0.0, 0.181, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_pivot_boss",
    )
    housing.visual(
        Cylinder(radius=0.023, length=0.028),
        origin=Origin(xyz=(0.0, -0.181, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_pivot_boss",
    )

    front_bar_span = opening + 0.008
    front_bar_height = opening + 0.008
    for index, y in enumerate((-0.09, -0.045, 0.0, 0.045, 0.09)):
        housing.visual(
            Box((grille_thickness, 0.006, front_bar_height)),
            origin=Origin(xyz=(grille_front_x, y, 0.0)),
            material=steel,
            name=f"front_vertical_bar_{index}",
        )
    for index, z in enumerate((-0.09, -0.045, 0.0, 0.045, 0.09)):
        housing.visual(
            Box((grille_thickness, front_bar_span, 0.006)),
            origin=Origin(xyz=(grille_front_x, 0.0, z)),
            material=steel,
            name=f"front_horizontal_bar_{index}",
        )

    rear_bar_span = opening + 0.008
    rear_bar_height = opening + 0.008
    for index, y in enumerate((-0.09, -0.045, 0.0, 0.045, 0.09)):
        housing.visual(
            Box((grille_thickness, 0.005, rear_bar_height)),
            origin=Origin(xyz=(grille_rear_x, y, 0.0)),
            material=steel,
            name=f"rear_vertical_bar_{index}",
        )
    for index, z in enumerate((-0.09, -0.045, 0.0, 0.045, 0.09)):
        housing.visual(
            Box((grille_thickness, rear_bar_span, 0.005)),
            origin=Origin(xyz=(grille_rear_x, 0.0, z)),
            material=steel,
            name=f"rear_horizontal_bar_{index}",
        )

    for index, z in enumerate((-0.065, 0.0, 0.065)):
        housing.visual(
            Box((0.05, 0.010, 0.010)),
            origin=Origin(xyz=(-0.022, 0.0, z)),
            material=charcoal,
            name=f"motor_strut_{index}",
        )
    housing.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(-0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="motor_can",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="shaft_stator",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.12, 0.38, 0.38)),
        mass=1.9,
        origin=Origin(),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft_axle",
    )
    for index in range(5):
        angle = index * (2.0 * math.pi / 5.0)
        rotor.visual(
            Box((0.006, 0.105, 0.038)),
            origin=Origin(
                xyz=(0.013, 0.058 * math.cos(angle), 0.058 * math.sin(angle)),
                rpy=(angle, 0.16, 0.0),
            ),
            material=trim_gray,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Box((0.03, 0.24, 0.24)),
        mass=0.22,
        origin=Origin(),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_shaft",
    )
    control_knob.visual(
        Cylinder(radius=0.019, length=0.026),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_body",
    )
    control_knob.visual(
        Box((0.004, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.036, 0.016)),
        material=body_white,
        name="knob_pointer",
    )
    control_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.030),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_housing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=math.radians(-28.0),
            upper=math.radians(22.0),
        ),
    )
    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=35.0,
        ),
    )
    model.articulation(
        "housing_to_control_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=control_knob,
        origin=Origin(xyz=(0.0, 0.192, -0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    control_knob = object_model.get_part("control_knob")
    tilt = object_model.get_articulation("base_to_housing")
    rotor_spin = object_model.get_articulation("housing_to_rotor")
    knob_spin = object_model.get_articulation("housing_to_control_knob")
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.check(
        "tilt joint uses bounded revolute motion",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0 < tilt.motion_limits.upper,
        details=f"limits={tilt.motion_limits}",
    )
    ctx.check(
        "blade and knob joints are continuous",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"rotor={rotor_spin.articulation_type}, knob={knob_spin.articulation_type}",
    )

    ctx.expect_gap(
        base,
        housing,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem="right_pivot_ear",
        negative_elem="right_pivot_boss",
        name="right pivot boss seats against right ear",
    )
    ctx.expect_gap(
        housing,
        base,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem="left_pivot_boss",
        negative_elem="left_pivot_ear",
        name="left pivot boss seats against left ear",
    )
    ctx.expect_gap(
        housing,
        rotor,
        axis="x",
        min_gap=0.008,
        positive_elem="front_top_frame",
        name="rotor clears the front grille",
    )
    ctx.expect_gap(
        rotor,
        housing,
        axis="x",
        min_gap=0.03,
        negative_elem="rear_top_frame",
        name="rotor clears the rear grille",
    )
    ctx.expect_contact(
        rotor,
        housing,
        elem_a="shaft_axle",
        elem_b="shaft_stator",
        contact_tol=1e-6,
        name="rotor shaft seats in the motor support",
    )
    ctx.expect_gap(
        control_knob,
        housing,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        negative_elem="knob_bushing",
        name="control knob mounts flush to the side bushing",
    )

    rest_front = ctx.part_element_world_aabb(housing, elem="front_top_frame")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        upper_front = ctx.part_element_world_aabb(housing, elem="front_top_frame")
    with ctx.pose({tilt: tilt.motion_limits.lower}):
        lower_front = ctx.part_element_world_aabb(housing, elem="front_top_frame")
    ctx.check(
        "fan tilts upward and downward around the base yoke",
        rest_front is not None
        and upper_front is not None
        and lower_front is not None
        and upper_front[1][2] > rest_front[1][2] + 0.004
        and lower_front[1][2] < rest_front[1][2] - 0.008,
        details=f"rest={rest_front}, upper={upper_front}, lower={lower_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
