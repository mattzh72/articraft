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
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_mouth_soup_blender")

    base_body = model.material("base_body", rgba=(0.18, 0.19, 0.21, 1.0))
    base_trim = model.material("base_trim", rgba=(0.78, 0.79, 0.80, 1.0))
    jug_glass = model.material("jug_glass", rgba=(0.82, 0.90, 0.94, 0.34))
    steel = model.material("steel", rgba=(0.75, 0.77, 0.79, 1.0))
    lid_black = model.material("lid_black", rgba=(0.11, 0.12, 0.13, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.83, 0.84, 0.86, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.120, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=base_body,
        name="foot_ring",
    )
    base.visual(
        Cylinder(radius=0.112, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=base_body,
        name="motor_housing",
    )
    base.visual(
        Cylinder(radius=0.088, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=base_trim,
        name="upper_trim",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=base_trim,
        name="drive_seat",
    )
    base.visual(
        Box((0.090, 0.010, 0.022)),
        origin=Origin(xyz=(0.080, 0.0, 0.044)),
        material=base_trim,
        name="control_band",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.120, length=0.110),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    jug = model.part("jug")
    jug_shell_mesh = _mesh(
        "jug_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.026, 0.000),
                (0.068, 0.000),
                (0.073, 0.010),
                (0.078, 0.028),
                (0.080, 0.180),
                (0.081, 0.236),
                (0.083, 0.256),
                (0.084, 0.264),
            ],
            [
                (0.000, 0.006),
                (0.058, 0.010),
                (0.068, 0.026),
                (0.071, 0.180),
                (0.073, 0.236),
                (0.076, 0.252),
                (0.078, 0.260),
            ],
            segments=72,
        ),
    )
    jug_handle_mesh = _mesh(
        "jug_handle",
        tube_from_spline_points(
            [
                (0.016, 0.073, 0.060),
                (0.040, 0.124, 0.092),
                (0.047, 0.132, 0.152),
                (0.034, 0.118, 0.214),
                (0.012, 0.074, 0.240),
            ],
            radius=0.008,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    jug.visual(jug_shell_mesh, material=jug_glass, name="jug_shell")
    jug.visual(jug_handle_mesh, material=lid_black, name="jug_handle")
    jug.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="lock_collar",
    )
    jug.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=steel,
        name="bearing_pillar",
    )
    jug.visual(
        Box((0.020, 0.034, 0.018)),
        origin=Origin(xyz=(0.088, 0.0, 0.257)),
        material=jug_glass,
        name="spout_nose",
    )
    jug.visual(
        Box((0.012, 0.012, 0.014)),
        origin=Origin(xyz=(-0.081, 0.031, 0.278)),
        material=steel,
        name="hinge_ear_left",
    )
    jug.visual(
        Box((0.012, 0.012, 0.014)),
        origin=Origin(xyz=(-0.081, -0.031, 0.278)),
        material=steel,
        name="hinge_ear_right",
    )
    jug.visual(
        Box((0.014, 0.052, 0.010)),
        origin=Origin(xyz=(-0.088, 0.0, 0.266)),
        material=steel,
        name="hinge_bridge",
    )
    jug.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.282),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.079, length=0.008),
        origin=Origin(xyz=(0.080, 0.0, -0.010)),
        material=lid_black,
        name="lid_cover",
    )
    lid.visual(
        Box((0.026, 0.034, 0.010)),
        origin=Origin(xyz=(0.157, 0.0, -0.011)),
        material=lid_black,
        name="lid_spout_flap",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=Origin(xyz=(0.004, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.080, 0.0, -0.001)),
        material=lid_black,
        name="lid_grip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.170, 0.170, 0.028)),
        mass=0.18,
        origin=Origin(xyz=(0.082, 0.0, -0.006)),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="blade_hub",
    )
    blade.visual(
        Cylinder(radius=0.005, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=steel,
        name="blade_shaft",
    )
    blade.visual(
        Box((0.050, 0.008, 0.002)),
        origin=Origin(xyz=(0.025, 0.0, 0.017), rpy=(0.0, 0.24, 0.12)),
        material=blade_steel,
        name="blade_primary",
    )
    blade.visual(
        Box((0.050, 0.008, 0.002)),
        origin=Origin(xyz=(-0.025, 0.0, 0.018), rpy=(0.0, -0.22, math.pi + 0.08)),
        material=blade_steel,
        name="blade_secondary",
    )
    blade.visual(
        Box((0.038, 0.008, 0.002)),
        origin=Origin(xyz=(0.0, 0.019, 0.024), rpy=(0.20, 0.0, math.pi / 2.0 + 0.14)),
        material=blade_steel,
        name="blade_upper_left",
    )
    blade.visual(
        Box((0.034, 0.008, 0.002)),
        origin=Origin(xyz=(0.0, -0.017, 0.013), rpy=(-0.18, 0.0, -math.pi / 2.0 + 0.06)),
        material=blade_steel,
        name="blade_lower_right",
    )
    blade.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=steel,
        name="blade_nut",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.040),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    twist_limit = math.radians(32.0)
    lid_limit = math.radians(112.0)

    model.articulation(
        "base_to_jug_twist",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jug,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=twist_limit,
        ),
    )
    model.articulation(
        "jug_to_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=jug,
        child=lid,
        origin=Origin(xyz=(-0.080, 0.0, 0.278)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=lid_limit,
        ),
    )
    model.articulation(
        "jug_to_blade_spin",
        ArticulationType.CONTINUOUS,
        parent=jug,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=30.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    jug = object_model.get_part("jug")
    lid = object_model.get_part("lid")
    blade = object_model.get_part("blade")

    twist = object_model.get_articulation("base_to_jug_twist")
    lid_hinge = object_model.get_articulation("jug_to_lid_hinge")
    blade_spin = object_model.get_articulation("jug_to_blade_spin")

    ctx.check(
        "twist mount is vertical revolute",
        twist.articulation_type == ArticulationType.REVOLUTE
        and tuple(twist.axis) == (0.0, 0.0, 1.0)
        and twist.motion_limits is not None
        and twist.motion_limits.upper is not None
        and twist.motion_limits.upper <= math.radians(40.0),
        details=f"type={twist.articulation_type}, axis={twist.axis}, limits={twist.motion_limits}",
    )
    ctx.check(
        "lid hinge opens upward around rear axis",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(lid_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}",
    )
    ctx.check(
        "blade spins continuously about vertical axis",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(blade_spin.axis) == (0.0, 0.0, 1.0)
        and blade_spin.motion_limits is not None
        and blade_spin.motion_limits.lower is None
        and blade_spin.motion_limits.upper is None,
        details=f"type={blade_spin.articulation_type}, axis={blade_spin.axis}, limits={blade_spin.motion_limits}",
    )

    with ctx.pose({twist: 0.0, lid_hinge: 0.0}):
        ctx.expect_gap(
            jug,
            base,
            axis="z",
            positive_elem="lock_collar",
            negative_elem="drive_seat",
            max_gap=0.0015,
            max_penetration=0.0,
            name="jug collar seats on the base drive seat",
        )
        ctx.expect_overlap(
            jug,
            base,
            axes="xy",
            elem_a="lock_collar",
            elem_b="drive_seat",
            min_overlap=0.100,
            name="jug remains centered over the round base",
        )
        ctx.expect_gap(
            lid,
            jug,
            axis="z",
            positive_elem="lid_cover",
            negative_elem="jug_shell",
            max_gap=0.003,
            max_penetration=0.0,
            name="closed lid sits on the jug rim",
        )
        ctx.expect_within(
            blade,
            jug,
            axes="xy",
            inner_elem="blade_primary",
            outer_elem="jug_shell",
            margin=0.010,
            name="blade stays within the jug footprint",
        )

    spout_rest = _aabb_center(ctx.part_element_world_aabb(jug, elem="spout_nose"))
    with ctx.pose({twist: twist.motion_limits.upper}):
        spout_twisted = _aabb_center(ctx.part_element_world_aabb(jug, elem="spout_nose"))
    ctx.check(
        "jug twist mount visibly rotates the jug",
        spout_rest is not None
        and spout_twisted is not None
        and spout_twisted[1] > spout_rest[1] + 0.015,
        details=f"rest={spout_rest}, twisted={spout_twisted}",
    )

    lid_closed = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_cover"))
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_open = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_cover"))
    ctx.check(
        "lid opens upward in the poured position",
        lid_closed is not None
        and lid_open is not None
        and lid_open[2] > lid_closed[2] + 0.045,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    blade_rest = _aabb_center(ctx.part_element_world_aabb(blade, elem="blade_primary"))
    with ctx.pose({blade_spin: 1.15}):
        blade_turned = _aabb_center(ctx.part_element_world_aabb(blade, elem="blade_primary"))
    ctx.check(
        "blade spin changes blade orientation around the vertical axis",
        blade_rest is not None
        and blade_turned is not None
        and blade_turned[1] > blade_rest[1] + 0.010,
        details=f"rest={blade_rest}, turned={blade_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
