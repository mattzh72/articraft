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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_desktop_turntable")

    charcoal = model.material("charcoal", rgba=(0.13, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    satin_black = model.material("satin_black", rgba=(0.07, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    warm_metal = model.material("warm_metal", rgba=(0.66, 0.58, 0.42, 1.0))

    plinth_width = 0.31
    plinth_depth = 0.22
    plinth_height = 0.036

    platter_center = (-0.050, 0.0)
    platter_radius = 0.095
    platter_joint_z = 0.038

    tonearm_pivot = (0.088, 0.050)
    tonearm_joint_z = 0.056
    arm_rest_xy = (0.106, -0.090)

    plinth_shell = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(plinth_width, plinth_depth, 0.020, corner_segments=10),
            plinth_height,
        ),
        "plinth_shell",
    )

    tonearm_tube = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, -0.003, 0.011),
                (0.005, -0.030, 0.013),
                (0.012, -0.085, 0.014),
                (0.020, -0.135, 0.011),
            ],
            radius=0.0032,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
        "tonearm_tube",
    )

    plinth = model.part("plinth")
    plinth.visual(plinth_shell, material=charcoal, name="top_deck")
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.115, -0.078),
            (0.115, -0.078),
            (-0.115, 0.078),
            (0.115, 0.078),
        )
    ):
        plinth.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.003)),
            material=rubber,
            name=f"foot_{index}",
        )
    plinth.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(-0.122, -0.080, plinth_height + 0.002)),
        material=warm_metal,
        name="speed_knob",
    )
    plinth.visual(
        Box((0.018, 0.012, 0.003)),
        origin=Origin(xyz=(-0.095, -0.079, plinth_height + 0.0015)),
        material=graphite,
        name="start_switch",
    )
    plinth.visual(
        Cylinder(radius=0.024, length=0.002),
        origin=Origin(xyz=(platter_center[0], platter_center[1], plinth_height + 0.001)),
        material=graphite,
        name="platter_flange",
    )
    plinth.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(platter_center[0], platter_center[1], 0.031)),
        material=graphite,
        name="bearing_sleeve",
    )
    plinth.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(platter_center[0], platter_center[1], 0.037)),
        material=aluminum,
        name="thrust_pad",
    )
    plinth.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], 0.043)),
        material=graphite,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], 0.050)),
        material=aluminum,
        name="tonearm_pedestal_top",
    )
    plinth.visual(
        Cylinder(radius=0.005, length=0.026),
        origin=Origin(xyz=(arm_rest_xy[0], arm_rest_xy[1], 0.049)),
        material=aluminum,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.020, 0.008, 0.005)),
        origin=Origin(xyz=(arm_rest_xy[0], arm_rest_xy[1], 0.0615)),
        material=graphite,
        name="arm_rest_pad",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((plinth_width, plinth_depth, 0.062)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=platter_radius, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0090)),
        material=aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.009, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=graphite,
        name="platter_underside_boss",
    )
    platter.visual(
        Cylinder(radius=0.084, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=satin_black,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.020, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0180)),
        material=warm_metal,
        name="label_disc",
    )
    platter.visual(
        Cylinder(radius=0.0016, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0230)),
        material=aluminum,
        name="spindle_pin",
    )
    platter.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=graphite,
        name="center_hub",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=platter_radius, length=0.020),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    tonearm_stage = model.part("tonearm_stage")
    tonearm_stage.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=aluminum,
        name="pivot_collar",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.018, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=graphite,
        name="stage_mount",
    )
    tonearm_stage.visual(
        Box((0.028, 0.018, 0.005)),
        origin=Origin(xyz=(0.002, -0.012, 0.0075)),
        material=graphite,
        name="arm_base",
    )
    tonearm_stage.visual(tonearm_tube, material=aluminum, name="arm_tube")
    tonearm_stage.visual(
        Box((0.018, 0.012, 0.003)),
        origin=Origin(xyz=(0.020, -0.141544, 0.010)),
        material=satin_black,
        name="headshell",
    )
    tonearm_stage.visual(
        Box((0.010, 0.014, 0.004)),
        origin=Origin(xyz=(0.018, -0.132, 0.011)),
        material=aluminum,
        name="headshell_socket",
    )
    tonearm_stage.visual(
        Box((0.010, 0.007, 0.005)),
        origin=Origin(xyz=(0.023, -0.147, 0.0060)),
        material=warm_metal,
        name="stylus_mount",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(xyz=(-0.003, 0.015, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="counterweight_stem",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(-0.003, 0.032, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="counterweight",
    )
    tonearm_stage.inertial = Inertial.from_geometry(
        Box((0.060, 0.200, 0.030)),
        mass=0.28,
        origin=Origin(xyz=(0.004, -0.060, 0.012)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_center[0], platter_center[1], platter_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm_stage,
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], tonearm_joint_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm_stage")
    arm_joint = object_model.get_articulation("tonearm_swing")

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
        platter,
        plinth,
        elem_a="platter_underside_boss",
        elem_b="thrust_pad",
        name="platter boss seats on thrust pad",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_underside_boss",
        elem_b="bearing_sleeve",
        min_overlap=0.014,
        name="platter boss stays centered in bearing sleeve footprint",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="top_deck",
        min_gap=0.004,
        max_gap=0.0085,
        name="platter clears plinth deck",
    )
    ctx.expect_contact(
        tonearm,
        plinth,
        elem_a="pivot_collar",
        elem_b="tonearm_pedestal_top",
        name="tonearm collar seats on pedestal",
    )

    with ctx.pose({arm_joint: 0.0}):
        ctx.expect_gap(
            tonearm,
            plinth,
            axis="z",
            positive_elem="headshell",
            negative_elem="arm_rest_pad",
            min_gap=0.0,
            max_gap=0.0015,
            name="stowed headshell hovers just above rest",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="x",
            positive_elem="headshell",
            negative_elem="platter_body",
            min_gap=0.030,
            name="stowed tonearm clears platter edge",
        )

    with ctx.pose({arm_joint: 1.10}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="headshell",
            elem_b="record_mat",
            min_overlap=0.010,
            name="tonearm reaches playable record area",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="stylus_mount",
            negative_elem="record_mat",
            min_gap=0.001,
            max_gap=0.008,
            name="stylus clears record plane in sweep pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
