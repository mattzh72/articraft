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
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    samples_per_segment: int = 14,
    radial_segments: int = 20,
):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def _side_path(
    path_xz: list[tuple[float, float]],
    *,
    y: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, z in path_xz]


def _aabb_center_x(aabb) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][0] + aabb[1][0])


def _aabb_center_z(aabb) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rocking_lounge_chair")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    shell_cream = model.material("shell_cream", rgba=(0.86, 0.83, 0.76, 1.0))
    cushion_taupe = model.material("cushion_taupe", rgba=(0.58, 0.53, 0.47, 1.0))
    matte_charcoal = model.material("matte_charcoal", rgba=(0.24, 0.25, 0.27, 1.0))

    rocker_base = model.part("rocker_base")
    rocker_base.inertial = Inertial.from_geometry(
        Box((1.12, 0.78, 0.44)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    rocker_path = [
        (-0.30, 0.23),
        (-0.40, 0.14),
        (-0.50, 0.040),
        (-0.22, 0.014),
        (0.14, 0.014),
        (0.46, 0.038),
        (0.31, 0.14),
        (0.22, 0.22),
    ]
    rear_strut_path = [(-0.30, 0.23), (-0.20, 0.29), (-0.02, 0.37)]
    front_strut_path = [(0.22, 0.22), (0.14, 0.30), (-0.01, 0.37)]

    for side_name, side_y in (("left", 0.34), ("right", -0.34)):
        rocker_base.visual(
            _tube_mesh(
                f"{side_name}_rocker_loop",
                _side_path(rocker_path, y=side_y),
                radius=0.018,
            ),
            material=graphite,
            name=f"{side_name}_rocker_loop",
        )
        rocker_base.visual(
            _tube_mesh(
                f"{side_name}_rear_strut",
                _side_path(rear_strut_path, y=side_y),
                radius=0.015,
                samples_per_segment=16,
            ),
            material=graphite,
            name=f"{side_name}_rear_strut",
        )
        rocker_base.visual(
            _tube_mesh(
                f"{side_name}_front_strut",
                _side_path(front_strut_path, y=side_y),
                radius=0.015,
                samples_per_segment=16,
            ),
            material=graphite,
            name=f"{side_name}_front_strut",
        )
        rocker_base.visual(
            Box((0.040, 0.032, 0.085)),
            origin=Origin(xyz=(-0.01, side_y, 0.372)),
            material=satin_black,
            name=f"{side_name}_pivot_block",
        )

    for name, x, z, radius, length in (
        ("front_spreader", 0.18, 0.19, 0.013, 0.66),
        ("rear_spreader", -0.24, 0.22, 0.013, 0.66),
        ("upper_spreader", -0.12, 0.300, 0.012, 0.72),
    ):
        rocker_base.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, 0.0, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=name,
        )
    for name, x, z, sx, sz in (
        ("front_bridge", 0.18, 0.19, 0.065, 0.055),
        ("rear_bridge", -0.24, 0.22, 0.065, 0.055),
        ("upper_bridge", -0.12, 0.300, 0.060, 0.040),
    ):
        rocker_base.visual(
            Box((sx, 0.72, sz)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=graphite,
            name=name,
        )

    seat_shell = model.part("seat_shell")
    seat_shell.inertial = Inertial.from_geometry(
        Box((0.62, 0.70, 0.30)),
        mass=8.5,
        origin=Origin(xyz=(0.08, 0.0, 0.12)),
    )
    seat_shell.visual(
        Cylinder(radius=0.018, length=0.58),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=matte_charcoal,
        name="rock_axis_tube",
    )
    seat_shell.visual(
        Box((0.060, 0.64, 0.130)),
        origin=Origin(xyz=(0.00, 0.0, 0.010)),
        material=matte_charcoal,
        name="pivot_sleeve_block",
    )
    for side_name, side_y in (("left", 0.317), ("right", -0.317)):
        seat_shell.visual(
            Box((0.040, 0.014, 0.085)),
            origin=Origin(xyz=(-0.010, side_y, -0.028)),
            material=matte_charcoal,
            name=f"{side_name}_rocker_bearing",
        )
    seat_shell.visual(
        Box((0.26, 0.56, 0.055)),
        origin=Origin(xyz=(0.10, 0.0, 0.045), rpy=(0.0, 0.10, 0.0)),
        material=matte_charcoal,
        name="underseat_bridge",
    )
    seat_shell.visual(
        Box((0.52, 0.60, 0.022)),
        origin=Origin(xyz=(0.08, 0.0, 0.104), rpy=(0.0, 0.12, 0.0)),
        material=shell_cream,
        name="seat_pan",
    )
    seat_shell.visual(
        Box((0.060, 0.58, 0.060)),
        origin=Origin(xyz=(0.31, 0.0, 0.105), rpy=(0.0, 0.12, 0.0)),
        material=shell_cream,
        name="seat_front_lip",
    )
    seat_shell.visual(
        Box((0.060, 0.56, 0.050)),
        origin=Origin(xyz=(-0.17, 0.0, 0.128)),
        material=matte_charcoal,
        name="rear_hinge_beam",
    )
    seat_shell.visual(
        Box((0.050, 0.56, 0.045)),
        origin=Origin(xyz=(0.34, 0.0, 0.052)),
        material=matte_charcoal,
        name="front_hinge_beam",
    )
    for side_name, side_y in (("left", 0.295), ("right", -0.295)):
        seat_shell.visual(
            Box((0.42, 0.028, 0.150)),
            origin=Origin(xyz=(0.07, side_y, 0.105), rpy=(0.0, 0.12, 0.0)),
            material=shell_cream,
            name=f"{side_name}_shell_side",
        )
        seat_shell.visual(
            Box((0.070, 0.045, 0.135)),
            origin=Origin(xyz=(-0.17, side_y, 0.155)),
            material=matte_charcoal,
            name=f"{side_name}_back_hinge_cheek",
        )
        seat_shell.visual(
            Box((0.060, 0.040, 0.095)),
            origin=Origin(xyz=(0.34, side_y, 0.078)),
            material=matte_charcoal,
            name=f"{side_name}_footrest_cheek",
        )

    backrest = model.part("backrest")
    backrest.inertial = Inertial.from_geometry(
        Box((0.22, 0.60, 0.72)),
        mass=4.6,
        origin=Origin(xyz=(-0.10, 0.0, 0.34)),
    )
    backrest.visual(
        Box((0.048, 0.56, 0.650)),
        origin=Origin(xyz=(-0.155, 0.0, 0.345), rpy=(0.0, -0.26, 0.0)),
        material=shell_cream,
        name="back_panel",
    )
    backrest.visual(
        Box((0.082, 0.48, 0.560)),
        origin=Origin(xyz=(-0.125, 0.0, 0.340), rpy=(0.0, -0.26, 0.0)),
        material=cushion_taupe,
        name="back_cushion",
    )
    backrest.visual(
        Box((0.050, 0.62, 0.040)),
        origin=Origin(xyz=(-0.085, 0.0, 0.010)),
        material=matte_charcoal,
        name="back_lower_beam",
    )
    for side_name, side_y in (("left", 0.245), ("right", -0.245)):
        backrest.visual(
            Box((0.110, 0.055, 0.080)),
            origin=Origin(xyz=(-0.055, side_y, 0.070)),
            material=matte_charcoal,
            name=f"{side_name}_back_hinge_lug",
        )

    footrest = model.part("footrest")
    footrest.inertial = Inertial.from_geometry(
        Box((0.40, 0.56, 0.12)),
        mass=2.9,
        origin=Origin(xyz=(0.18, 0.0, -0.03)),
    )
    footrest.visual(
        Box((0.36, 0.52, 0.020)),
        origin=Origin(xyz=(0.19, 0.0, -0.030), rpy=(0.0, -0.34, 0.0)),
        material=shell_cream,
        name="foot_panel",
    )
    footrest.visual(
        Box((0.040, 0.52, 0.040)),
        origin=Origin(xyz=(0.045, 0.0, 0.002)),
        material=matte_charcoal,
        name="foot_hinge_leaf",
    )
    footrest.visual(
        Box((0.24, 0.12, 0.055)),
        origin=Origin(xyz=(0.16, 0.0, -0.048), rpy=(0.0, -0.34, 0.0)),
        material=matte_charcoal,
        name="foot_center_rib",
    )
    for side_name, side_y in (("left", 0.255), ("right", -0.255)):
        footrest.visual(
            Box((0.28, 0.020, 0.070)),
            origin=Origin(xyz=(0.15, side_y, -0.020), rpy=(0.0, -0.34, 0.0)),
            material=matte_charcoal,
            name=f"{side_name}_foot_side",
        )

    model.articulation(
        "rocking_joint",
        ArticulationType.REVOLUTE,
        parent=rocker_base,
        child=seat_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.8,
            lower=-0.24,
            upper=0.30,
        ),
    )
    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=backrest,
        origin=Origin(xyz=(-0.17, 0.0, 0.135)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=0.58,
        ),
    )
    model.articulation(
        "footrest_hinge",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=footrest,
        origin=Origin(xyz=(0.34, 0.0, 0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=-0.25,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rocker_base = object_model.get_part("rocker_base")
    seat_shell = object_model.get_part("seat_shell")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")
    rocking_joint = object_model.get_articulation("rocking_joint")
    backrest_hinge = object_model.get_articulation("backrest_hinge")
    footrest_hinge = object_model.get_articulation("footrest_hinge")

    ctx.expect_origin_gap(
        seat_shell,
        rocker_base,
        axis="z",
        min_gap=0.38,
        max_gap=0.42,
        name="seat shell is mounted above the rocker frame",
    )
    ctx.expect_origin_gap(
        seat_shell,
        backrest,
        axis="x",
        min_gap=0.15,
        max_gap=0.19,
        name="backrest hinge line sits at the rear of the seat shell",
    )
    ctx.expect_origin_gap(
        footrest,
        seat_shell,
        axis="x",
        min_gap=0.32,
        max_gap=0.36,
        name="footrest hinge line sits at the front edge of the seat shell",
    )
    ctx.expect_origin_distance(
        backrest,
        footrest,
        axes="y",
        min_dist=0.0,
        max_dist=0.001,
        name="backrest and footrest stay centered on the chair width",
    )
    ctx.expect_contact(
        backrest,
        seat_shell,
        name="backrest hinge hardware contacts the seat shell",
    )
    ctx.expect_contact(
        footrest,
        seat_shell,
        name="footrest hinge hardware contacts the seat shell",
    )

    front_lip_rest = ctx.part_element_world_aabb(seat_shell, elem="seat_front_lip")
    with ctx.pose({rocking_joint: 0.24}):
        front_lip_rocked = ctx.part_element_world_aabb(seat_shell, elem="seat_front_lip")
    front_lip_rest_z = _aabb_center_z(front_lip_rest)
    front_lip_rocked_z = _aabb_center_z(front_lip_rocked)
    ctx.check(
        "positive rocking pose lifts the front lip",
        front_lip_rest_z is not None
        and front_lip_rocked_z is not None
        and front_lip_rocked_z > front_lip_rest_z + 0.05,
        details=f"rest_z={front_lip_rest_z}, rocked_z={front_lip_rocked_z}",
    )

    back_rest = ctx.part_element_world_aabb(backrest, elem="back_panel")
    with ctx.pose({backrest_hinge: 0.50}):
        back_reclined = ctx.part_element_world_aabb(backrest, elem="back_panel")
    back_rest_x = _aabb_center_x(back_rest)
    back_reclined_x = _aabb_center_x(back_reclined)
    ctx.check(
        "backrest reclines rearward",
        back_rest_x is not None
        and back_reclined_x is not None
        and back_reclined_x < back_rest_x - 0.08,
        details=f"rest_x={back_rest_x}, reclined_x={back_reclined_x}",
    )

    foot_rest = ctx.part_element_world_aabb(footrest, elem="foot_panel")
    with ctx.pose({footrest_hinge: 0.45}):
        foot_raised = ctx.part_element_world_aabb(footrest, elem="foot_panel")
    foot_rest_z = _aabb_center_z(foot_rest)
    foot_raised_z = _aabb_center_z(foot_raised)
    ctx.check(
        "footrest raises independently",
        foot_rest_z is not None
        and foot_raised_z is not None
        and foot_raised_z > foot_rest_z + 0.07,
        details=f"rest_z={foot_rest_z}, raised_z={foot_raised_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
