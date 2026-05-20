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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _shell_tube(radius_outer: float, radius_inner: float, length: float, name: str):
    half = length * 0.5
    outer_profile = ((radius_outer, -half), (radius_outer, half))
    inner_profile = ((radius_inner, -half), (radius_inner, half))
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_site_floodlight")

    safety_yellow = model.material("safety_yellow", rgba=(0.93, 0.79, 0.14, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.60, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.77, 0.88, 0.96, 0.42))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.070, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=dark_gray,
        name="tripod_hub",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=dark_gray,
        name="lower_socket",
    )
    base.visual(
        _shell_tube(0.023, 0.0195, 0.620, "outer_tube_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        material=steel,
        name="outer_tube",
    )
    base.visual(
        _shell_tube(0.032, 0.0205, 0.100, "upper_sleeve_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        material=dark_gray,
        name="upper_sleeve",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(xyz=(0.040, 0.0, 0.900), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="clamp_stem",
    )
    base.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(0.062, 0.0, 0.900)),
        material=rubber,
        name="clamp_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.035 * c, 0.035 * s, 0.140),
                (0.165 * c, 0.165 * s, 0.095),
                (0.560 * c, 0.560 * s, 0.025),
            ],
            radius=0.014,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        base.visual(
            mesh_from_geometry(leg_mesh, f"tripod_leg_{index}"),
            material=safety_yellow,
            name=f"leg_{index}",
        )
        base.visual(
            Cylinder(radius=0.028, length=0.018),
            origin=Origin(xyz=(0.560 * c, 0.560 * s, 0.016)),
            material=rubber,
            name=f"foot_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((1.24, 1.24, 1.00)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
    )

    inner_mast = model.part("inner_mast")
    inner_mast.visual(
        Cylinder(radius=0.018, length=1.340),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=steel,
        name="inner_tube",
    )
    inner_mast.visual(
        Cylinder(radius=0.029, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=dark_gray,
        name="stop_collar",
    )
    inner_mast.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
        material=dark_gray,
        name="mast_cap",
    )
    inner_mast.visual(
        Box((0.080, 0.100, 0.090)),
        origin=Origin(xyz=(0.0, -0.005, 0.815)),
        material=dark_gray,
        name="yoke_neck",
    )
    inner_mast.visual(
        Box((0.060, 0.050, 0.060)),
        origin=Origin(xyz=(0.0, -0.010, 0.875)),
        material=dark_gray,
        name="yoke_stanchion",
    )
    inner_mast.visual(
        Box((0.460, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, -0.025, 0.925)),
        material=dark_gray,
        name="yoke_crossbar",
    )
    inner_mast.visual(
        Box((0.036, 0.050, 0.160)),
        origin=Origin(xyz=(-0.230, -0.010, 0.980)),
        material=dark_gray,
        name="left_yoke_arm",
    )
    inner_mast.visual(
        Box((0.036, 0.050, 0.160)),
        origin=Origin(xyz=(0.230, -0.010, 0.980)),
        material=dark_gray,
        name="right_yoke_arm",
    )
    inner_mast.inertial = Inertial.from_geometry(
        Box((0.620, 0.220, 1.700)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -0.010, 0.290)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.016, length=0.422),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pivot_shaft",
    )
    lamp_head.visual(
        Box((0.026, 0.048, 0.120)),
        origin=Origin(xyz=(-0.192, 0.030, 0.0)),
        material=safety_yellow,
        name="left_trunnion_boss",
    )
    lamp_head.visual(
        Box((0.026, 0.048, 0.120)),
        origin=Origin(xyz=(0.192, 0.030, 0.0)),
        material=safety_yellow,
        name="right_trunnion_boss",
    )
    lamp_head.visual(
        Box((0.420, 0.110, 0.280)),
        origin=Origin(xyz=(0.0, 0.095, 0.0)),
        material=safety_yellow,
        name="housing",
    )
    lamp_head.visual(
        Box((0.390, 0.022, 0.250)),
        origin=Origin(xyz=(0.0, 0.156, 0.0)),
        material=dark_gray,
        name="bezel",
    )
    lamp_head.visual(
        Box((0.350, 0.004, 0.210)),
        origin=Origin(xyz=(0.0, 0.169, 0.0)),
        material=lens_glass,
        name="front_glass",
    )
    lamp_head.visual(
        Box((0.350, 0.034, 0.210)),
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
        material=dark_gray,
        name="rear_cap",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.460, 0.160, 0.340)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.090, 0.0)),
    )

    model.articulation(
        "base_to_inner_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.880)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=0.18,
            lower=0.0,
            upper=0.360,
        ),
    )
    model.articulation(
        "inner_mast_to_lamp_head",
        ArticulationType.REVOLUTE,
        parent=inner_mast,
        child=lamp_head,
        origin=Origin(xyz=(0.0, -0.005, 1.000)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=-0.450,
            upper=1.250,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    inner_mast = object_model.get_part("inner_mast")
    lamp_head = object_model.get_part("lamp_head")
    mast_slide = object_model.get_articulation("base_to_inner_mast")
    tilt_joint = object_model.get_articulation("inner_mast_to_lamp_head")
    mast_upper = (
        mast_slide.motion_limits.upper
        if mast_slide.motion_limits is not None and mast_slide.motion_limits.upper is not None
        else 0.360
    )

    ctx.expect_within(
        inner_mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_tube",
        margin=0.0,
        name="inner mast stays centered in outer sleeve",
    )
    ctx.expect_overlap(
        inner_mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_tube",
        min_overlap=0.50,
        name="collapsed mast remains deeply inserted",
    )

    rest_pos = ctx.part_world_position(inner_mast)
    rest_glass_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_glass")
    rest_glass_z = None
    if rest_glass_aabb is not None:
        rest_glass_z = (rest_glass_aabb[0][2] + rest_glass_aabb[1][2]) * 0.5

    with ctx.pose({mast_slide: mast_upper}):
        ctx.expect_within(
            inner_mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_tube",
            margin=0.0,
            name="extended mast stays centered in outer sleeve",
        )
        ctx.expect_overlap(
            inner_mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_tube",
            min_overlap=0.17,
            name="extended mast retains insertion in sleeve",
        )
        extended_pos = ctx.part_world_position(inner_mast)

    ctx.check(
        "mast extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({tilt_joint: 0.90}):
        tilted_glass_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_glass")
        tilted_glass_z = None
        if tilted_glass_aabb is not None:
            tilted_glass_z = (tilted_glass_aabb[0][2] + tilted_glass_aabb[1][2]) * 0.5
        ctx.check(
            "lamp head tilts downward",
            rest_glass_z is not None
            and tilted_glass_z is not None
            and tilted_glass_z < rest_glass_z - 0.05,
            details=f"rest_glass_z={rest_glass_z}, tilted_glass_z={tilted_glass_z}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
