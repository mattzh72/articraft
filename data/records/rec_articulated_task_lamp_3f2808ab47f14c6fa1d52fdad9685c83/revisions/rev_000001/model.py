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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _tube_shell(name: str, *, outer_radius: float, inner_radius: float, length: float):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, length)],
            [(inner_radius, 0.0), (inner_radius, length)],
            segments=48,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_gooseneck_desk_lamp")

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    warm_white = model.material("warm_white", rgba=(0.90, 0.90, 0.87, 1.0))
    soft_rubber = model.material("soft_rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    base_length = 0.220
    base_width = 0.140
    foot_height = 0.005
    base_height = 0.028
    base_top = foot_height + base_height
    mast_x = -0.070

    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(base_length, base_width, 0.018), base_height),
        "lamp_base_plate",
    )
    base = model.part("base")
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, foot_height)),
        material=matte_black,
        name="base_plate",
    )
    base.visual(
        Box((0.090, 0.070, 0.018)),
        origin=Origin(xyz=(mast_x + 0.010, 0.0, base_top - 0.009)),
        material=satin_graphite,
        name="counterweight_block",
    )
    base.visual(
        Box((0.050, 0.040, 0.012)),
        origin=Origin(xyz=(mast_x, 0.0, base_top + 0.006)),
        material=satin_graphite,
        name="mast_plinth",
    )
    for index, (x_sign, y_sign) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        base.visual(
            Cylinder(radius=0.010, length=foot_height),
            origin=Origin(
                xyz=(0.078 * x_sign, 0.046 * y_sign, foot_height * 0.5),
            ),
            material=soft_rubber,
            name=f"foot_pad_{index}",
        )

    base_receiver_len = 0.160
    base.visual(
        _tube_shell(
            "base_receiver_shell",
            outer_radius=0.0188,
            inner_radius=0.0166,
            length=base_receiver_len,
        ),
        origin=Origin(xyz=(mast_x, 0.0, base_top)),
        material=brushed_aluminum,
        name="base_receiver",
    )
    base.visual(
        Cylinder(radius=0.0225, length=0.016),
        origin=Origin(xyz=(mast_x, 0.0, base_top + 0.012)),
        material=satin_graphite,
        name="base_collar_lower",
    )
    base.visual(
        _tube_shell(
            "base_collar_upper_ring",
            outer_radius=0.0210,
            inner_radius=0.0169,
            length=0.020,
        ),
        origin=Origin(xyz=(mast_x, 0.0, base_top + base_receiver_len - 0.020)),
        material=satin_graphite,
        name="base_collar_upper",
    )
    base.visual(
        Cylinder(radius=0.0045, length=0.026),
        origin=Origin(
            xyz=(mast_x + 0.023, 0.0, base_top + 0.122),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_graphite,
        name="base_tension_knob",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.240, 0.160, 0.220)),
        mass=3.8,
        origin=Origin(xyz=(-0.020, 0.0, 0.110)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        _tube_shell(
            "lower_stage_shell",
            outer_radius=0.0150,
            inner_radius=0.0130,
            length=0.270,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=brushed_aluminum,
        name="lower_sleeve",
    )
    lower_stage.visual(
        _tube_shell(
            "lower_collar_bottom_ring",
            outer_radius=0.0180,
            inner_radius=0.0144,
            length=0.016,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_graphite,
        name="lower_collar_bottom",
    )
    lower_stage.visual(
        _tube_shell(
            "lower_collar_top_ring",
            outer_radius=0.0170,
            inner_radius=0.0125,
            length=0.018,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=satin_graphite,
        name="lower_collar_top",
    )
    lower_stage.visual(
        Cylinder(radius=0.0040, length=0.022),
        origin=Origin(
            xyz=(0.0185, 0.0, 0.086),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_graphite,
        name="lower_tension_knob",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.290)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        _tube_shell(
            "middle_stage_shell",
            outer_radius=0.0118,
            inner_radius=0.0101,
            length=0.240,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=brushed_aluminum,
        name="middle_sleeve",
    )
    middle_stage.visual(
        _tube_shell(
            "middle_collar_bottom_ring",
            outer_radius=0.0138,
            inner_radius=0.0110,
            length=0.014,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_graphite,
        name="middle_collar_bottom",
    )
    middle_stage.visual(
        _tube_shell(
            "middle_collar_top_ring",
            outer_radius=0.0130,
            inner_radius=0.0098,
            length=0.018,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=satin_graphite,
        name="middle_collar_top",
    )
    middle_stage.inertial = Inertial.from_geometry(
        Box((0.032, 0.032, 0.255)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        Cylinder(radius=0.0087, length=0.208),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=brushed_aluminum,
        name="upper_mast",
    )
    upper_stage.visual(
        Cylinder(radius=0.0116, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=satin_graphite,
        name="upper_stop_collar",
    )
    upper_stage.visual(
        Box((0.030, 0.020, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=satin_graphite,
        name="shoulder_mount_block",
    )
    upper_stage.inertial = Inertial.from_geometry(
        Box((0.032, 0.024, 0.230)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.0105, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="shoulder_barrel",
    )
    lower_arm.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.004, 0.0, 0.000),
                    (0.095, 0.0, 0.010),
                    (0.170, 0.0, 0.016),
                    (0.214, 0.0, 0.020),
                ],
                radius=0.0066,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "lower_arm_tube",
        ),
        material=matte_black,
        name="lower_arm_tube",
    )
    lower_arm.visual(
        Box((0.022, 0.016, 0.016)),
        origin=Origin(xyz=(0.2155, 0.0, 0.015)),
        material=satin_graphite,
        name="lower_arm_tip_block",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.250, 0.040, 0.045)),
        mass=0.24,
        origin=Origin(xyz=(0.115, 0.0, 0.010)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.0095, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="elbow_barrel",
    )
    upper_arm.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.004, 0.0, 0.000),
                    (0.086, 0.0, 0.008),
                    (0.148, 0.0, 0.012),
                    (0.190, 0.0, 0.015),
                ],
                radius=0.0058,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "upper_arm_tube",
        ),
        material=matte_black,
        name="upper_arm_tube",
    )
    upper_arm.visual(
        Box((0.020, 0.014, 0.014)),
        origin=Origin(xyz=(0.190, 0.0, 0.011)),
        material=satin_graphite,
        name="upper_arm_tip_block",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.220, 0.035, 0.040)),
        mass=0.19,
        origin=Origin(xyz=(0.098, 0.0, 0.008)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.0100, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="shade_barrel",
    )
    shade.visual(
        Cylinder(radius=0.0135, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_graphite,
        name="shade_neck",
    )
    shade.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.018, 0.0), (0.028, 0.024), (0.055, 0.105)],
                [(0.012, 0.0), (0.021, 0.024), (0.049, 0.105)],
                segments=56,
            ),
            "lamp_shade_shell",
        ),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lamp_socket",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.125, 0.120, 0.090)),
        mass=0.18,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(mast_x, 0.0, base_top + base_receiver_len)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.10,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "lower_to_middle_stage",
        ArticulationType.PRISMATIC,
        parent=lower_stage,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.10,
            lower=0.0,
            upper=0.060,
        ),
    )
    model.articulation(
        "middle_to_upper_stage",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.10,
            lower=0.0,
            upper=0.060,
        ),
    )
    model.articulation(
        "upper_stage_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=upper_stage,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.45,
            upper=1.15,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.236, 0.0, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-1.40,
            upper=1.40,
        ),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(0.210, 0.0, 0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-0.80,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    middle_stage = object_model.get_part("middle_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    base_to_lower = object_model.get_articulation("base_to_lower_stage")
    lower_to_middle = object_model.get_articulation("lower_to_middle_stage")
    middle_to_upper = object_model.get_articulation("middle_to_upper_stage")
    shoulder = object_model.get_articulation("upper_stage_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    shade_tilt = object_model.get_articulation("upper_arm_to_shade")

    def elem_center(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    with ctx.pose({base_to_lower: 0.0, lower_to_middle: 0.0, middle_to_upper: 0.0}):
        ctx.expect_within(
            lower_stage,
            base,
            axes="xy",
            inner_elem="lower_sleeve",
            outer_elem="base_receiver",
            margin=0.0025,
            name="lower stage stays centered in base receiver at rest",
        )
        ctx.expect_overlap(
            lower_stage,
            base,
            axes="z",
            elem_a="lower_sleeve",
            elem_b="base_receiver",
            min_overlap=0.050,
            name="lower stage remains inserted in base receiver at rest",
        )
        ctx.expect_within(
            middle_stage,
            lower_stage,
            axes="xy",
            inner_elem="middle_sleeve",
            outer_elem="lower_sleeve",
            margin=0.0025,
            name="middle stage stays centered in lower stage at rest",
        )
        ctx.expect_overlap(
            middle_stage,
            lower_stage,
            axes="z",
            elem_a="middle_sleeve",
            elem_b="lower_sleeve",
            min_overlap=0.050,
            name="middle stage remains inserted in lower stage at rest",
        )
        ctx.expect_within(
            upper_stage,
            middle_stage,
            axes="xy",
            inner_elem="upper_mast",
            outer_elem="middle_sleeve",
            margin=0.0020,
            name="upper stage stays centered in middle stage at rest",
        )
        ctx.expect_overlap(
            upper_stage,
            middle_stage,
            axes="z",
            elem_a="upper_mast",
            elem_b="middle_sleeve",
            min_overlap=0.040,
            name="upper stage remains inserted in middle stage at rest",
        )

    rest_upper_stage_pos = ctx.part_world_position(upper_stage)
    with ctx.pose(
        {
            base_to_lower: 0.080,
            lower_to_middle: 0.060,
            middle_to_upper: 0.060,
        }
    ):
        ctx.expect_overlap(
            lower_stage,
            base,
            axes="z",
            elem_a="lower_sleeve",
            elem_b="base_receiver",
            min_overlap=0.045,
            name="lower stage retains insertion at full extension",
        )
        ctx.expect_overlap(
            middle_stage,
            lower_stage,
            axes="z",
            elem_a="middle_sleeve",
            elem_b="lower_sleeve",
            min_overlap=0.045,
            name="middle stage retains insertion at full extension",
        )
        ctx.expect_overlap(
            upper_stage,
            middle_stage,
            axes="z",
            elem_a="upper_mast",
            elem_b="middle_sleeve",
            min_overlap=0.038,
            name="upper stage retains insertion at full extension",
        )
        extended_upper_stage_pos = ctx.part_world_position(upper_stage)

    ctx.check(
        "telescoping post extends upward",
        rest_upper_stage_pos is not None
        and extended_upper_stage_pos is not None
        and extended_upper_stage_pos[2] > rest_upper_stage_pos[2] + 0.18,
        details=f"rest={rest_upper_stage_pos}, extended={extended_upper_stage_pos}",
    )

    rest_shade_center = elem_center(shade, "shade_shell")
    with ctx.pose({shoulder: 0.55, elbow: 0.75}):
        raised_shade_center = elem_center(shade, "shade_shell")
    ctx.check(
        "arm links raise the shade upward",
        rest_shade_center is not None
        and raised_shade_center is not None
        and raised_shade_center[2] > rest_shade_center[2] + 0.08,
        details=f"rest={rest_shade_center}, raised={raised_shade_center}",
    )

    with ctx.pose({shoulder: 0.35, elbow: 0.45, shade_tilt: 0.0}):
        level_shade_center = elem_center(shade, "shade_shell")
    with ctx.pose({shoulder: 0.35, elbow: 0.45, shade_tilt: 0.45}):
        tilted_shade_center = elem_center(shade, "shade_shell")
    ctx.check(
        "shade tilt pitches the head upward",
        level_shade_center is not None
        and tilted_shade_center is not None
        and tilted_shade_center[2] > level_shade_center[2] + 0.015,
        details=f"level={level_shade_center}, tilted={tilted_shade_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
