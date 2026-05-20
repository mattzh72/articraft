from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
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
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gooseneck_floor_lamp")

    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_nickel = model.material("satin_nickel", rgba=(0.72, 0.74, 0.76, 1.0))
    shade_white = model.material("shade_white", rgba=(0.93, 0.93, 0.91, 1.0))

    base_profile = [
        (0.0, 0.0),
        (0.110, 0.0),
        (0.145, 0.004),
        (0.160, 0.013),
        (0.165, 0.028),
        (0.160, 0.039),
        (0.148, 0.045),
        (0.0, 0.045),
    ]
    base = model.part("base")
    base.visual(
        _mesh("lamp_base", LatheGeometry(base_profile, segments=72)),
        material=graphite,
        name="base_disk",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.165, length=0.045),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
    )

    column_height = 1.42
    column = model.part("column")
    column.visual(
        Cylinder(radius=0.028, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=graphite,
        name="lower_collar",
    )
    column.visual(
        Cylinder(radius=0.014, length=1.36),
        origin=Origin(xyz=(0.0, 0.0, 0.71)),
        material=satin_nickel,
        name="main_stem",
    )
    column.visual(
        Cylinder(radius=0.024, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 1.405)),
        material=satin_nickel,
        name="upper_cap",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=column_height),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, column_height * 0.5)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    arm_tip = (0.57, 0.0, -0.135)
    gooseneck_path = [
        (0.0, 0.0, 0.015),
        (0.05, 0.0, 0.135),
        (0.16, 0.0, 0.235),
        (0.32, 0.0, 0.255),
        (0.45, 0.0, 0.155),
        (0.505, 0.0, 0.015),
        (0.536, 0.0, -0.135),
    ]
    arm = model.part("arm")
    arm.visual(
        _mesh(
            "gooseneck_tube",
            tube_from_spline_points(
                gooseneck_path,
                radius=0.012,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=satin_nickel,
        name="gooseneck_tube",
    )
    arm.visual(
        Cylinder(radius=0.026, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_nickel,
        name="arm_mount_collar",
    )
    arm.visual(
        Box((0.036, 0.018, 0.018)),
        origin=Origin(xyz=(arm_tip[0] - 0.022, 0.0, arm_tip[2])),
        material=satin_nickel,
        name="tip_connector",
    )
    arm.visual(
        Box((0.014, 0.004, 0.030)),
        origin=Origin(xyz=(arm_tip[0] + 0.003, 0.0105, arm_tip[2])),
        material=satin_nickel,
        name="tip_plate_left",
    )
    arm.visual(
        Box((0.014, 0.004, 0.030)),
        origin=Origin(xyz=(arm_tip[0] + 0.003, -0.0105, arm_tip[2])),
        material=satin_nickel,
        name="tip_plate_right",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.60, 0.06, 0.42)),
        mass=2.0,
        origin=Origin(xyz=(0.30, 0.0, 0.05)),
    )

    model.articulation(
        "column_to_arm",
        ArticulationType.FIXED,
        parent=column,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, column_height)),
    )

    shade_outer = [
        (0.018, 0.018),
        (0.040, 0.014),
        (0.074, -0.015),
        (0.108, -0.070),
        (0.112, -0.118),
    ]
    shade_inner = [
        (0.0, 0.012),
        (0.024, 0.012),
        (0.058, -0.018),
        (0.090, -0.070),
        (0.098, -0.118),
    ]
    shade = model.part("shade")
    shade.visual(
        Box((0.020, 0.020, 0.028)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=satin_nickel,
        name="shade_hub",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.049, 0.0, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_nickel,
        name="shade_neck",
    )
    shade.visual(
        _mesh(
            "shade_shell",
            LatheGeometry.from_shell_profiles(shade_outer, shade_inner, segments=64),
        ),
        origin=Origin(xyz=(0.095, 0.0, -0.028)),
        material=shade_white,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.15),
        mass=0.9,
        origin=Origin(xyz=(0.095, 0.0, -0.07)),
    )

    model.articulation(
        "arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=arm_tip),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=math.radians(-55.0),
            upper=math.radians(35.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    arm = object_model.get_part("arm")
    shade = object_model.get_part("shade")
    shade_tilt = object_model.get_articulation("arm_to_shade")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(column, base, elem_a="lower_collar", elem_b="base_disk")
    ctx.expect_contact(arm, column, elem_a="arm_mount_collar", elem_b="upper_cap")
    ctx.expect_contact(shade, arm, elem_a="shade_hub", elem_b="tip_plate_left")

    ctx.expect_overlap(column, base, axes="xy", min_overlap=0.05)
    ctx.expect_overlap(arm, column, axes="xy", elem_a="arm_mount_collar", elem_b="upper_cap", min_overlap=0.04)

    limits = shade_tilt.motion_limits
    ctx.check(
        "shade_tilt_axis_and_limits",
        shade_tilt.axis == (0.0, 1.0, 0.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        f"axis={shade_tilt.axis}, limits={limits}",
    )

    base_aabb = ctx.part_world_aabb(base)
    arm_aabb = ctx.part_world_aabb(arm)
    assert base_aabb is not None
    assert arm_aabb is not None
    lamp_height = arm_aabb[1][2] - base_aabb[0][2]
    ctx.check(
        "floor_lamp_height_plausible",
        1.60 <= lamp_height <= 1.90,
        f"expected floor-lamp height in [1.60, 1.90] m, got {lamp_height:.3f} m",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    assert rest_shell_aabb is not None
    rest_center = _aabb_center(rest_shell_aabb)
    with ctx.pose({shade_tilt: math.radians(25.0)}):
        posed_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
        assert posed_shell_aabb is not None
        posed_center = _aabb_center(posed_shell_aabb)
        dx = abs(posed_center[0] - rest_center[0])
        dy = abs(posed_center[1] - rest_center[1])
        dz = abs(posed_center[2] - rest_center[2])
        ctx.expect_contact(shade, arm, elem_a="shade_hub", elem_b="tip_plate_left")
        ctx.check(
            "shade_tilt_moves_in_pitch_plane",
            dx > 0.012 and dz > 0.012 and dy < 0.003,
            f"expected tilt to move shade mostly in x/z; got dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
