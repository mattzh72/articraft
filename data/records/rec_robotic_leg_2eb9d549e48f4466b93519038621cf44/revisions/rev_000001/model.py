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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    housing_gray = model.material("housing_gray", rgba=(0.66, 0.69, 0.72, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.18, 0.20, 0.23, 1.0))
    link_black = model.material("link_black", rgba=(0.13, 0.14, 0.15, 1.0))
    joint_steel = model.material("joint_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    sole_rubber = model.material("sole_rubber", rgba=(0.08, 0.09, 0.10, 1.0))

    upper_housing = model.part("upper_housing")
    housing_geom = BoxGeometry((0.130, 0.140, 0.035)).translate(0.010, 0.0, 0.105)
    housing_geom.merge(BoxGeometry((0.055, 0.085, 0.085)).translate(-0.035, 0.0, 0.070))
    housing_geom.merge(BoxGeometry((0.028, 0.108, 0.060)).translate(0.052, 0.0, 0.030))
    housing_geom.merge(BoxGeometry((0.070, 0.016, 0.145)).translate(0.010, 0.054, 0.045))
    housing_geom.merge(BoxGeometry((0.070, 0.016, 0.145)).translate(0.010, -0.054, 0.045))
    housing_geom.merge(
        CylinderGeometry(radius=0.024, height=0.024, radial_segments=24)
        .rotate_x(math.pi / 2.0)
        .translate(0.000, 0.066, 0.000)
    )
    housing_geom.merge(
        CylinderGeometry(radius=0.024, height=0.024, radial_segments=24)
        .rotate_x(math.pi / 2.0)
        .translate(0.000, -0.066, 0.000)
    )
    housing_geom.merge(
        CylinderGeometry(radius=0.020, height=0.090, radial_segments=24)
        .rotate_y(math.pi / 2.0)
        .translate(0.030, 0.0, 0.084)
    )
    upper_housing.visual(
        _mesh("upper_housing_frame", housing_geom),
        material=housing_gray,
        name="housing_frame",
    )
    upper_housing.visual(
        Box((0.088, 0.060, 0.048)),
        origin=Origin(xyz=(0.028, 0.0, 0.145)),
        material=dark_graphite,
        name="controller_pack",
    )
    upper_housing.inertial = Inertial.from_geometry(
        Box((0.190, 0.160, 0.190)),
        mass=7.5,
        origin=Origin(xyz=(0.010, 0.0, 0.085)),
    )

    thigh_link = model.part("thigh_link")
    thigh_link.visual(
        Box((0.050, 0.092, 0.050)),
        origin=Origin(xyz=(0.000, 0.0, 0.000)),
        material=joint_steel,
        name="hip_trunnion",
    )
    thigh_link.visual(
        Box((0.016, 0.016, 0.255)),
        origin=Origin(xyz=(0.028, 0.026, -0.150)),
        material=link_black,
        name="thigh_rail_left",
    )
    thigh_link.visual(
        Box((0.016, 0.016, 0.255)),
        origin=Origin(xyz=(0.028, -0.026, -0.150)),
        material=link_black,
        name="thigh_rail_right",
    )
    thigh_link.visual(
        Box((0.022, 0.056, 0.018)),
        origin=Origin(xyz=(0.022, 0.0, -0.075)),
        material=link_black,
        name="thigh_upper_tie",
    )
    thigh_link.visual(
        Box((0.018, 0.056, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, -0.175)),
        material=link_black,
        name="thigh_mid_tie",
    )
    thigh_link.visual(
        Box((0.018, 0.042, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, -0.250)),
        material=link_black,
        name="thigh_lower_tie",
    )
    thigh_link.visual(
        Box((0.016, 0.016, 0.090)),
        origin=Origin(xyz=(0.018, 0.024, -0.290)),
        material=link_black,
        name="knee_cheek_left",
    )
    thigh_link.visual(
        Box((0.016, 0.016, 0.090)),
        origin=Origin(xyz=(0.018, -0.024, -0.290)),
        material=link_black,
        name="knee_cheek_right",
    )
    thigh_link.visual(
        Box((0.020, 0.040, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, -0.329)),
        material=joint_steel,
        name="knee_mount_block",
    )
    thigh_link.visual(
        Box((0.046, 0.030, 0.024)),
        origin=Origin(xyz=(0.020, 0.0, -0.037)),
        material=joint_steel,
        name="hip_actuator_pack",
    )
    thigh_link.inertial = Inertial.from_geometry(
        Box((0.120, 0.120, 0.390)),
        mass=4.4,
        origin=Origin(xyz=(0.020, 0.0, -0.185)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_housing,
        child=thigh_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.2,
            lower=-0.75,
            upper=0.95,
        ),
    )

    shank_link = model.part("shank_link")
    shank_link.visual(
        Box((0.036, 0.030, 0.040)),
        origin=Origin(xyz=(0.000, 0.0, -0.020)),
        material=joint_steel,
        name="knee_yoke",
    )
    shank_link.visual(
        Box((0.014, 0.014, 0.115)),
        origin=Origin(xyz=(0.018, 0.022, -0.060)),
        material=link_black,
        name="shank_upper_rail_left",
    )
    shank_link.visual(
        Box((0.014, 0.014, 0.115)),
        origin=Origin(xyz=(0.018, -0.022, -0.060)),
        material=link_black,
        name="shank_upper_rail_right",
    )
    shank_link.visual(
        Box((0.014, 0.014, 0.185)),
        origin=Origin(xyz=(0.024, 0.022, -0.202)),
        material=link_black,
        name="shank_lower_rail_left",
    )
    shank_link.visual(
        Box((0.014, 0.014, 0.185)),
        origin=Origin(xyz=(0.024, -0.022, -0.202)),
        material=link_black,
        name="shank_lower_rail_right",
    )
    shank_link.visual(
        Box((0.018, 0.046, 0.016)),
        origin=Origin(xyz=(0.022, 0.0, -0.115)),
        material=link_black,
        name="shank_mid_tie",
    )
    shank_link.visual(
        Box((0.016, 0.050, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, -0.220)),
        material=link_black,
        name="shank_lower_tie",
    )
    shank_link.visual(
        Box((0.014, 0.014, 0.060)),
        origin=Origin(xyz=(0.018, 0.028, -0.282)),
        material=link_black,
        name="ankle_cheek_left",
    )
    shank_link.visual(
        Box((0.014, 0.014, 0.060)),
        origin=Origin(xyz=(0.018, -0.028, -0.282)),
        material=link_black,
        name="ankle_cheek_right",
    )
    shank_link.visual(
        Box((0.020, 0.056, 0.014)),
        origin=Origin(xyz=(0.018, 0.0, -0.284)),
        material=joint_steel,
        name="ankle_mount_block",
    )
    shank_link.inertial = Inertial.from_geometry(
        Box((0.100, 0.110, 0.350)),
        mass=3.4,
        origin=Origin(xyz=(0.018, 0.0, -0.170)),
    )

    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.020, 0.0, -0.335)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=190.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    foot_module = model.part("foot_module")
    foot_module.visual(
        Box((0.034, 0.044, 0.036)),
        origin=Origin(xyz=(0.000, 0.0, -0.018)),
        material=joint_steel,
        name="ankle_yoke",
    )
    foot_module.visual(
        Box((0.018, 0.044, 0.012)),
        origin=Origin(xyz=(0.000, 0.0, -0.006)),
        material=joint_steel,
        name="ankle_top_block",
    )
    foot_module.visual(
        Box((0.014, 0.012, 0.070)),
        origin=Origin(xyz=(0.018, 0.020, -0.040)),
        material=link_black,
        name="foot_strut_left",
    )
    foot_module.visual(
        Box((0.014, 0.012, 0.070)),
        origin=Origin(xyz=(0.018, -0.020, -0.040)),
        material=link_black,
        name="foot_strut_right",
    )
    foot_module.visual(
        Box((0.110, 0.028, 0.022)),
        origin=Origin(xyz=(0.060, 0.0, -0.072)),
        material=link_black,
        name="forefoot_beam",
    )
    foot_module.visual(
        Box((0.046, 0.024, 0.018)),
        origin=Origin(xyz=(-0.014, 0.0, -0.072)),
        material=link_black,
        name="heel_beam",
    )
    foot_module.visual(
        Box((0.118, 0.024, 0.028)),
        origin=Origin(xyz=(0.030, 0.0, -0.092)),
        material=dark_graphite,
        name="sole_supports",
    )
    foot_module.visual(
        Box((0.170, 0.062, 0.016)),
        origin=Origin(xyz=(0.058, 0.0, -0.112)),
        material=sole_rubber,
        name="main_sole_pad",
    )
    foot_module.visual(
        Box((0.040, 0.050, 0.012)),
        origin=Origin(xyz=(-0.030, 0.0, -0.110)),
        material=sole_rubber,
        name="heel_pad",
    )
    foot_module.inertial = Inertial.from_geometry(
        Box((0.200, 0.080, 0.140)),
        mass=1.5,
        origin=Origin(xyz=(0.040, 0.0, -0.082)),
    )

    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=foot_module,
        origin=Origin(xyz=(0.018, 0.0, -0.312)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.8,
            lower=-0.65,
            upper=0.55,
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

    upper_housing = object_model.get_part("upper_housing")
    thigh_link = object_model.get_part("thigh_link")
    shank_link = object_model.get_part("shank_link")
    foot_module = object_model.get_part("foot_module")
    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    housing_aabb = ctx.part_world_aabb(upper_housing)
    thigh_aabb = ctx.part_world_aabb(thigh_link)
    ctx.check(
        "thigh hangs below the housing",
        housing_aabb is not None
        and thigh_aabb is not None
        and thigh_aabb[0][2] < housing_aabb[0][2] - 0.22
        and thigh_aabb[1][2] < housing_aabb[1][2] - 0.06,
        details=f"housing_aabb={housing_aabb}, thigh_aabb={thigh_aabb}",
    )

    rest_aabb = thigh_aabb
    with ctx.pose({hip_pitch: 0.55}):
        flexed_aabb = ctx.part_world_aabb(thigh_link)
    rest_center_x = None if rest_aabb is None else 0.5 * (rest_aabb[0][0] + rest_aabb[1][0])
    flexed_center_x = None if flexed_aabb is None else 0.5 * (flexed_aabb[0][0] + flexed_aabb[1][0])
    ctx.check(
        "positive hip rotation pitches the thigh forward",
        rest_center_x is not None and flexed_center_x is not None and flexed_center_x > rest_center_x + 0.06,
        details=f"rest_center_x={rest_center_x}, flexed_center_x={flexed_center_x}",
    )

    knee_pos = ctx.part_world_position(shank_link)
    ankle_pos = ctx.part_world_position(foot_module)
    ctx.check(
        "knee and ankle descend below the hip in the neutral pose",
        knee_pos is not None
        and ankle_pos is not None
        and knee_pos[2] < -0.30
        and ankle_pos[2] < knee_pos[2] - 0.26,
        details=f"knee_pos={knee_pos}, ankle_pos={ankle_pos}",
    )

    rest_ankle = ankle_pos
    with ctx.pose({knee_pitch: 1.0}):
        bent_ankle = ctx.part_world_position(foot_module)
    ctx.check(
        "positive knee rotation folds the shank rearward and upward",
        rest_ankle is not None
        and bent_ankle is not None
        and bent_ankle[0] < rest_ankle[0] - 0.10
        and bent_ankle[2] > rest_ankle[2] + 0.08,
        details=f"rest_ankle={rest_ankle}, bent_ankle={bent_ankle}",
    )

    rest_sole = ctx.part_element_world_aabb(foot_module, elem="main_sole_pad")
    with ctx.pose({ankle_pitch: 0.40}):
        dorsiflexed_sole = ctx.part_element_world_aabb(foot_module, elem="main_sole_pad")
    rest_sole_max_z = None if rest_sole is None else rest_sole[1][2]
    dorsiflexed_sole_max_z = None if dorsiflexed_sole is None else dorsiflexed_sole[1][2]
    ctx.check(
        "positive ankle rotation lifts the toe section",
        rest_sole_max_z is not None
        and dorsiflexed_sole_max_z is not None
        and dorsiflexed_sole_max_z > rest_sole_max_z + 0.02,
        details=f"rest_sole_max_z={rest_sole_max_z}, dorsiflexed_sole_max_z={dorsiflexed_sole_max_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
