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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tube_shell_mesh():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.035, -0.078),
            (0.034, -0.060),
            (0.031, -0.022),
            (0.031, 0.148),
            (0.034, 0.188),
        ],
        [
            (0.030, -0.078),
            (0.029, -0.060),
            (0.026, -0.022),
            (0.026, 0.145),
            (0.029, 0.184),
        ],
        segments=60,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_y(math.pi / 2.0)
    return shell


def _tripod_leg_mesh(angle: float):
    ca = math.cos(angle)
    sa = math.sin(angle)
    return tube_from_spline_points(
        [
            (0.020 * ca, 0.020 * sa, 0.120),
            (0.060 * ca, 0.060 * sa, 0.078),
            (0.108 * ca, 0.108 * sa, 0.010),
        ],
        radius=0.0065,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _spreader_mesh(angle: float):
    ca = math.cos(angle)
    sa = math.sin(angle)
    return tube_from_spline_points(
        [
            (0.012 * ca, 0.012 * sa, 0.088),
            (0.038 * ca, 0.038 * sa, 0.071),
            (0.066 * ca, 0.066 * sa, 0.060),
        ],
        radius=0.0035,
        samples_per_segment=8,
        radial_segments=14,
        cap_ends=True,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_first_telescope")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    brushed_aluminum = model.material(
        "brushed_aluminum", rgba=(0.73, 0.75, 0.78, 1.0)
    )
    tube_blue = model.material("tube_blue", rgba=(0.22, 0.35, 0.57, 1.0))
    soft_white = model.material("soft_white", rgba=(0.88, 0.89, 0.91, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=0.024, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=brushed_aluminum,
        name="center_column",
    )
    tripod_base.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.127)),
        material=charcoal,
        name="hub_block",
    )
    tripod_base.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=matte_black,
        name="bearing_cap",
    )
    tripod_base.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=charcoal,
        name="spreader_collar",
    )
    for index in range(3):
        angle = index * math.tau / 3.0
        tripod_base.visual(
            _save_mesh(f"tripod_leg_{index}", _tripod_leg_mesh(angle)),
            material=brushed_aluminum,
            name=f"tripod_leg_{index}",
        )
        tripod_base.visual(
            _save_mesh(f"spreader_{index}", _spreader_mesh(angle)),
            material=charcoal,
            name=f"spreader_{index}",
        )
        foot_x = 0.108 * math.cos(angle)
        foot_y = 0.108 * math.sin(angle)
        tripod_base.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(foot_x, foot_y, 0.005)),
            material=rubber,
            name=f"foot_{index}",
        )
    tripod_base.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.16)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    azimuth_head = model.part("azimuth_head")
    azimuth_head.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=matte_black,
        name="turntable_disk",
    )
    azimuth_head.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=charcoal,
        name="head_post",
    )
    azimuth_head.visual(
        Box((0.048, 0.090, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=charcoal,
        name="saddle_block",
    )
    azimuth_head.visual(
        Box((0.020, 0.010, 0.066)),
        origin=Origin(xyz=(0.0, 0.045, 0.109)),
        material=matte_black,
        name="left_fork_arm",
    )
    azimuth_head.visual(
        Box((0.020, 0.010, 0.066)),
        origin=Origin(xyz=(0.0, -0.045, 0.109)),
        material=matte_black,
        name="right_fork_arm",
    )
    azimuth_head.visual(
        Box((0.018, 0.090, 0.010)),
        origin=Origin(xyz=(-0.014, 0.0, 0.068)),
        material=matte_black,
        name="fork_brace",
    )
    azimuth_head.inertial = Inertial.from_geometry(
        Box((0.09, 0.10, 0.13)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    telescope_tube = model.part("telescope_tube")
    telescope_tube.visual(
        _save_mesh("tube_shell", _tube_shell_mesh()),
        material=tube_blue,
        name="tube_shell",
    )
    telescope_tube.visual(
        Cylinder(radius=0.036, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_white,
        name="tube_cradle_band",
    )
    telescope_tube.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.186, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_white,
        name="objective_cell",
    )
    telescope_tube.visual(
        Cylinder(radius=0.006, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="tube_trunnion",
    )
    telescope_tube.visual(
        Cylinder(radius=0.026, length=0.080),
        origin=Origin(xyz=(-0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_white,
        name="rear_casting",
    )
    telescope_tube.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(-0.093, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_white,
        name="focuser_body",
    )
    telescope_tube.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(-0.126, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="drawtube",
    )
    telescope_tube.visual(
        Cylinder(radius=0.014, length=0.042),
        origin=Origin(xyz=(-0.159, 0.0, 0.015), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=matte_black,
        name="diagonal_body",
    )
    telescope_tube.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(-0.177, 0.0, 0.053)),
        material=matte_black,
        name="eyepiece",
    )
    telescope_tube.inertial = Inertial.from_geometry(
        Box((0.38, 0.10, 0.12)),
        mass=0.65,
        origin=Origin(xyz=(0.040, 0.0, 0.010)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=tripod_base,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, 0.157)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )
    model.articulation(
        "altitude_tilt",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=telescope_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.4,
            lower=math.radians(-18.0),
            upper=math.radians(72.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_base = object_model.get_part("tripod_base")
    azimuth_head = object_model.get_part("azimuth_head")
    telescope_tube = object_model.get_part("telescope_tube")
    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    altitude_tilt = object_model.get_articulation("altitude_tilt")

    ctx.expect_contact(
        azimuth_head,
        tripod_base,
        elem_a="turntable_disk",
        elem_b="bearing_cap",
        name="azimuth turntable sits on tripod bearing",
    )
    ctx.expect_contact(
        telescope_tube,
        azimuth_head,
        elem_a="tube_trunnion",
        elem_b="left_fork_arm",
        name="tube trunnion bears on left fork arm",
    )
    ctx.expect_contact(
        telescope_tube,
        azimuth_head,
        elem_a="tube_trunnion",
        elem_b="right_fork_arm",
        name="tube trunnion bears on right fork arm",
    )

    rest_obj = _aabb_center(
        ctx.part_element_world_aabb(telescope_tube, elem="objective_cell")
    )
    with ctx.pose({altitude_tilt: math.radians(55.0)}):
        raised_obj = _aabb_center(
            ctx.part_element_world_aabb(telescope_tube, elem="objective_cell")
        )
    ctx.check(
        "altitude joint raises the telescope objective",
        rest_obj is not None
        and raised_obj is not None
        and raised_obj[2] > rest_obj[2] + 0.10
        and raised_obj[0] < rest_obj[0] - 0.03,
        details=f"rest={rest_obj}, raised={raised_obj}",
    )

    with ctx.pose({azimuth_rotation: 0.0}):
        az0_obj = _aabb_center(
            ctx.part_element_world_aabb(telescope_tube, elem="objective_cell")
        )
    with ctx.pose({azimuth_rotation: math.pi / 2.0}):
        az90_obj = _aabb_center(
            ctx.part_element_world_aabb(telescope_tube, elem="objective_cell")
        )
    ctx.check(
        "azimuth bearing swings the tube around the vertical axis",
        az0_obj is not None
        and az90_obj is not None
        and abs(az0_obj[0]) > 0.12
        and abs(az90_obj[1]) > 0.12
        and abs(az0_obj[2] - az90_obj[2]) < 0.01,
        details=f"az0={az0_obj}, az90={az90_obj}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
