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
)


LINK_LENGTH = 0.190
OUTER_WIDTH = 0.048
OUTER_HEIGHT = 0.050
SIDE_PLATE = 0.008
WEB_THICKNESS = 0.007
INNER_WIDTH = OUTER_WIDTH - 2.0 * SIDE_PLATE
INNER_HEIGHT = OUTER_HEIGHT - 2.0 * WEB_THICKNESS
JOINT_RADIUS = 0.015
BARREL_LENGTH = 0.026
FORK_CLEARANCE = 0.0
FORK_GAP = BARREL_LENGTH + 2.0 * FORK_CLEARANCE
EAR_THICKNESS = (OUTER_WIDTH - FORK_GAP) / 2.0
EAR_Y = FORK_GAP / 2.0 + EAR_THICKNESS / 2.0


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material,
    name: str,
):
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_y_cylinder_visual(
    part,
    *,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material,
    name: str,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_fork_features(part, *, prefix: str, axis_x: float, material) -> None:
    ear_center_x = (axis_x - 0.032 + axis_x + 0.015) / 2.0
    ear_length = 0.047
    for side_name, y_center in (("left", EAR_Y), ("right", -EAR_Y)):
        _add_box_visual(
            part,
            size=(ear_length, EAR_THICKNESS, OUTER_HEIGHT),
            center=(ear_center_x, y_center, 0.0),
            material=material,
            name=f"{prefix}_{side_name}_ear",
        )
        _add_y_cylinder_visual(
            part,
            radius=JOINT_RADIUS + 0.006,
            length=EAR_THICKNESS,
            center=(axis_x, y_center, 0.0),
            material=material,
            name=f"{prefix}_{side_name}_boss",
        )


def _add_long_link_visuals(part, *, prefix: str, material) -> None:
    beam_x0 = 0.030
    beam_x1 = LINK_LENGTH - 0.032
    beam_len = beam_x1 - beam_x0
    beam_center_x = (beam_x0 + beam_x1) / 2.0
    plate_y = OUTER_WIDTH / 2.0 - SIDE_PLATE / 2.0
    web_z = OUTER_HEIGHT / 2.0 - WEB_THICKNESS / 2.0
    bridge_y = BARREL_LENGTH / 2.0 + 0.0015

    _add_y_cylinder_visual(
        part,
        radius=JOINT_RADIUS,
        length=BARREL_LENGTH,
        center=(0.0, 0.0, 0.0),
        material=material,
        name=f"{prefix}_barrel",
    )
    _add_box_visual(
        part,
        size=(0.030, BARREL_LENGTH, 0.032),
        center=(0.020, 0.0, 0.0),
        material=material,
        name=f"{prefix}_neck",
    )
    for side_name, y_center in (("left", bridge_y), ("right", -bridge_y)):
        _add_box_visual(
            part,
            size=(0.026, 0.007, 0.040),
            center=(0.032, y_center, 0.0),
            material=material,
            name=f"{prefix}_{side_name}_bridge",
        )
    _add_box_visual(
        part,
        size=(beam_len, SIDE_PLATE, OUTER_HEIGHT),
        center=(beam_center_x, plate_y, 0.0),
        material=material,
        name=f"{prefix}_left_plate",
    )
    _add_box_visual(
        part,
        size=(beam_len, SIDE_PLATE, OUTER_HEIGHT),
        center=(beam_center_x, -plate_y, 0.0),
        material=material,
        name=f"{prefix}_right_plate",
    )
    _add_box_visual(
        part,
        size=(beam_len, INNER_WIDTH, WEB_THICKNESS),
        center=(beam_center_x, 0.0, web_z),
        material=material,
        name=f"{prefix}_top_web",
    )
    _add_box_visual(
        part,
        size=(beam_len, INNER_WIDTH, WEB_THICKNESS),
        center=(beam_center_x, 0.0, -web_z),
        material=material,
        name=f"{prefix}_bottom_web",
    )
    _add_fork_features(part, prefix=prefix, axis_x=LINK_LENGTH, material=material)


def _add_end_tab_visuals(part, *, prefix: str, material) -> None:
    _add_y_cylinder_visual(
        part,
        radius=JOINT_RADIUS,
        length=BARREL_LENGTH,
        center=(0.0, 0.0, 0.0),
        material=material,
        name=f"{prefix}_barrel",
    )
    _add_box_visual(
        part,
        size=(0.028, BARREL_LENGTH, 0.028),
        center=(0.018, 0.0, 0.0),
        material=material,
        name=f"{prefix}_neck",
    )
    _add_box_visual(
        part,
        size=(0.042, 0.022, 0.022),
        center=(0.053, 0.0, 0.0),
        material=material,
        name=f"{prefix}_tab_body",
    )
    _add_box_visual(
        part,
        size=(0.020, 0.032, 0.036),
        center=(0.080, 0.0, 0.0),
        material=material,
        name=f"{prefix}_tab_pad",
    )
    _add_y_cylinder_visual(
        part,
        radius=0.010,
        length=0.032,
        center=(0.091, 0.0, 0.0),
        material=material,
        name=f"{prefix}_tab_tip",
    )


def _add_root_bracket_visuals(part, *, prefix: str, material) -> None:
    _add_box_visual(
        part,
        size=(0.110, 0.070, 0.014),
        center=(-0.030, 0.0, -0.087),
        material=material,
        name=f"{prefix}_base_plate",
    )
    _add_box_visual(
        part,
        size=(0.024, 0.060, 0.118),
        center=(-0.055, 0.0, -0.031),
        material=material,
        name=f"{prefix}_rear_web",
    )
    _add_box_visual(
        part,
        size=(0.056, 0.050, 0.020),
        center=(-0.035, 0.0, -0.050),
        material=material,
        name=f"{prefix}_lower_crossbar",
    )
    _add_box_visual(
        part,
        size=(0.038, EAR_THICKNESS, OUTER_HEIGHT),
        center=(-0.028, EAR_Y, 0.0),
        material=material,
        name=f"{prefix}_left_shoulder",
    )
    _add_box_visual(
        part,
        size=(0.038, EAR_THICKNESS, OUTER_HEIGHT),
        center=(-0.028, -EAR_Y, 0.0),
        material=material,
        name=f"{prefix}_right_shoulder",
    )
    _add_fork_features(part, prefix=prefix, axis_x=0.0, material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_four_joint_revolute_chain")

    bracket_finish = model.material("bracket_finish", rgba=(0.22, 0.24, 0.27, 1.0))
    link_finish = model.material("link_finish", rgba=(0.71, 0.73, 0.76, 1.0))
    tip_finish = model.material("tip_finish", rgba=(0.18, 0.20, 0.23, 1.0))

    root_bracket = model.part("root_bracket")
    _add_root_bracket_visuals(root_bracket, prefix="root_bracket", material=bracket_finish)
    root_bracket.inertial = Inertial.from_geometry(
        Box((0.110, 0.070, 0.132)),
        mass=1.8,
        origin=Origin(xyz=(-0.030, 0.0, -0.032)),
    )

    link_1 = model.part("link_1")
    _add_long_link_visuals(link_1, prefix="link_1", material=link_finish)
    link_1.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH + 0.020, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=0.62,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    link_2 = model.part("link_2")
    _add_long_link_visuals(link_2, prefix="link_2", material=link_finish)
    link_2.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH + 0.020, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=0.62,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    link_3 = model.part("link_3")
    _add_long_link_visuals(link_3, prefix="link_3", material=link_finish)
    link_3.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH + 0.020, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=0.62,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    end_tab = model.part("end_tab")
    _add_end_tab_visuals(end_tab, prefix="end_tab", material=tip_finish)
    end_tab.inertial = Inertial.from_geometry(
        Box((0.105, 0.032, 0.036)),
        mass=0.20,
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
    )

    root_to_link_1 = model.articulation(
        "root_to_link_1",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.15, upper=1.20, effort=35.0, velocity=2.0),
    )
    link_1_to_link_2 = model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=25.0, velocity=2.5),
    )
    link_2_to_link_3 = model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=25.0, velocity=2.5),
    )
    model.articulation(
        "link_3_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=end_tab,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.55, upper=1.55, effort=18.0, velocity=3.0),
    )

    # Keep the first articulation object alive for static analyzers and local reasoning.
    _ = root_to_link_1, link_1_to_link_2, link_2_to_link_3
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
    root_bracket = object_model.get_part("root_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    end_tab = object_model.get_part("end_tab")

    root_to_link_1 = object_model.get_articulation("root_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")
    link_3_to_end_tab = object_model.get_articulation("link_3_to_end_tab")

    joints = (
        root_to_link_1,
        link_1_to_link_2,
        link_2_to_link_3,
        link_3_to_end_tab,
    )

    ctx.check(
        "four serial revolute joints use parallel pitch axes",
        all(joint.joint_type == ArticulationType.REVOLUTE and joint.axis == (0.0, -1.0, 0.0) for joint in joints),
        details=str([(joint.name, joint.joint_type, joint.axis) for joint in joints]),
    )
    ctx.check(
        "all revolute joints have bounded travel",
        all(
            joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower < 0.0 < joint.motion_limits.upper
            for joint in joints
        ),
        details=str(
            [
                (
                    joint.name,
                    None if joint.motion_limits is None else joint.motion_limits.lower,
                    None if joint.motion_limits is None else joint.motion_limits.upper,
                )
                for joint in joints
            ]
        ),
    )

    with ctx.pose(
        {
            root_to_link_1: 0.0,
            link_1_to_link_2: 0.0,
            link_2_to_link_3: 0.0,
            link_3_to_end_tab: 0.0,
        }
    ):
        ctx.expect_overlap(
            root_bracket,
            link_1,
            axes="yz",
            min_overlap=0.022,
            name="root bracket and first link stay aligned across the supported joint",
        )
        ctx.expect_overlap(
            link_1,
            link_2,
            axes="yz",
            min_overlap=0.024,
            name="first and second links stay aligned across the second joint",
        )
        ctx.expect_overlap(
            link_2,
            link_3,
            axes="yz",
            min_overlap=0.024,
            name="second and third links stay aligned across the third joint",
        )
        ctx.expect_overlap(
            link_3,
            end_tab,
            axes="yz",
            min_overlap=0.020,
            name="third link and end tab stay aligned across the tip joint",
        )
        ctx.expect_origin_gap(
            link_2,
            link_1,
            axis="x",
            min_gap=LINK_LENGTH - 0.001,
            max_gap=LINK_LENGTH + 0.001,
            name="second link origin sits one link length beyond the first",
        )
        ctx.expect_origin_gap(
            link_3,
            link_2,
            axis="x",
            min_gap=LINK_LENGTH - 0.001,
            max_gap=LINK_LENGTH + 0.001,
            name="third link origin sits one link length beyond the second",
        )
        ctx.expect_origin_gap(
            end_tab,
            link_3,
            axis="x",
            min_gap=LINK_LENGTH - 0.001,
            max_gap=LINK_LENGTH + 0.001,
            name="end tab origin sits one link length beyond the third link",
        )

        rest_link_2 = ctx.part_world_position(link_2)
        rest_link_3 = ctx.part_world_position(link_3)
        rest_end_tab = ctx.part_world_position(end_tab)

    with ctx.pose(
        {
            root_to_link_1: 0.60,
            link_1_to_link_2: 0.45,
            link_2_to_link_3: 0.30,
            link_3_to_end_tab: 0.20,
        }
    ):
        posed_link_2 = ctx.part_world_position(link_2)
        posed_link_3 = ctx.part_world_position(link_3)
        posed_end_tab = ctx.part_world_position(end_tab)

    ctx.check(
        "positive joint motion lifts the second link",
        rest_link_2 is not None
        and posed_link_2 is not None
        and posed_link_2[2] > rest_link_2[2] + 0.08,
        details=f"rest={rest_link_2}, posed={posed_link_2}",
    )
    ctx.check(
        "serial positive motion lifts the third link and tip",
        rest_link_3 is not None
        and rest_end_tab is not None
        and posed_link_3 is not None
        and posed_end_tab is not None
        and posed_link_3[2] > rest_link_3[2] + 0.15
        and posed_end_tab[2] > rest_end_tab[2] + 0.24,
        details=(
            f"rest_link_3={rest_link_3}, posed_link_3={posed_link_3}, "
            f"rest_end_tab={rest_end_tab}, posed_end_tab={posed_end_tab}"
        ),
    )
    ctx.check(
        "chain remains in a single y-centered motion plane",
        posed_link_2 is not None
        and posed_link_3 is not None
        and posed_end_tab is not None
        and max(abs(posed_link_2[1]), abs(posed_link_3[1]), abs(posed_end_tab[1])) < 1e-6,
        details=f"posed_link_2={posed_link_2}, posed_link_3={posed_link_3}, posed_end_tab={posed_end_tab}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
