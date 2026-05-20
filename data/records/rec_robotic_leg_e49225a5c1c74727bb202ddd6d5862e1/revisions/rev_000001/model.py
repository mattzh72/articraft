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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rect_section(
    size_x: float,
    size_y: float,
    z: float,
    *,
    radius: float,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
):
    return tuple(
        (x + x_offset, y + y_offset, z)
        for x, y in rounded_rect_profile(size_x, size_y, radius, corner_segments=8)
    )


def _lofted_shell(name: str, sections: list[tuple[float, float, float, float]]):
    return mesh_from_geometry(
        section_loft(
            [
                _rect_section(size_x, size_y, z, radius=radius)
                for size_x, size_y, z, radius in sections
            ]
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    housing_gray = model.material("housing_gray", rgba=(0.44, 0.47, 0.50, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    link_gray = model.material("link_gray", rgba=(0.54, 0.57, 0.60, 1.0))
    foot_dark = model.material("foot_dark", rgba=(0.22, 0.24, 0.26, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    thigh_shell_mesh = _lofted_shell(
        "thigh_shell",
        [
            (0.140, 0.082, -0.010, 0.018),
            (0.128, 0.076, -0.165, 0.016),
            (0.102, 0.062, -0.290, 0.012),
        ],
    )
    shank_shell_mesh = _lofted_shell(
        "shank_shell",
        [
            (0.108, 0.056, -0.010, 0.014),
            (0.092, 0.050, -0.145, 0.012),
            (0.078, 0.044, -0.250, 0.010),
        ],
    )

    upper_leg_housing = model.part("upper_leg_housing")
    upper_leg_housing.visual(
        Box((0.180, 0.160, 0.180)),
        origin=Origin(xyz=(0.000, 0.000, 0.150)),
        material=housing_gray,
        name="main_body",
    )
    upper_leg_housing.visual(
        Box((0.120, 0.110, 0.080)),
        origin=Origin(xyz=(-0.030, 0.000, 0.220)),
        material=dark_metal,
        name="upper_pack",
    )
    upper_leg_housing.visual(
        Box((0.105, 0.160, 0.030)),
        origin=Origin(xyz=(-0.010, 0.000, 0.065)),
        material=dark_metal,
        name="hip_bridge",
    )
    upper_leg_housing.visual(
        Box((0.070, 0.026, 0.140)),
        origin=Origin(xyz=(0.000, 0.067, -0.010)),
        material=housing_gray,
        name="left_hip_yoke",
    )
    upper_leg_housing.visual(
        Box((0.070, 0.026, 0.140)),
        origin=Origin(xyz=(0.000, -0.067, -0.010)),
        material=housing_gray,
        name="right_hip_yoke",
    )
    upper_leg_housing.visual(
        Box((0.155, 0.095, 0.050)),
        origin=Origin(xyz=(0.035, 0.000, 0.185)),
        material=housing_gray,
        name="front_cowl",
    )
    upper_leg_housing.inertial = Inertial.from_geometry(
        Box((0.200, 0.180, 0.310)),
        mass=10.5,
        origin=Origin(xyz=(0.000, 0.000, 0.105)),
    )

    thigh_link = model.part("thigh_link")
    thigh_link.visual(
        Cylinder(radius=0.038, length=0.108),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hip_hub",
    )
    thigh_link.visual(thigh_shell_mesh, material=link_gray, name="thigh_shell")
    thigh_link.visual(
        Box((0.092, 0.082, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, -0.285)),
        material=dark_metal,
        name="knee_bridge",
    )
    thigh_link.visual(
        Box((0.058, 0.018, 0.120)),
        origin=Origin(xyz=(0.000, 0.041, -0.335)),
        material=link_gray,
        name="left_knee_yoke",
    )
    thigh_link.visual(
        Box((0.058, 0.018, 0.120)),
        origin=Origin(xyz=(0.000, -0.041, -0.335)),
        material=link_gray,
        name="right_knee_yoke",
    )
    thigh_link.inertial = Inertial.from_geometry(
        Box((0.145, 0.090, 0.400)),
        mass=6.4,
        origin=Origin(xyz=(0.000, 0.000, -0.200)),
    )

    shank_link = model.part("shank_link")
    shank_link.visual(
        Cylinder(radius=0.032, length=0.064),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="knee_hub",
    )
    shank_link.visual(shank_shell_mesh, material=link_gray, name="shank_shell")
    shank_link.visual(
        Box((0.078, 0.070, 0.040)),
        origin=Origin(xyz=(0.000, 0.000, -0.255)),
        material=dark_metal,
        name="ankle_bridge",
    )
    shank_link.visual(
        Box((0.050, 0.016, 0.095)),
        origin=Origin(xyz=(0.000, 0.035, -0.3025)),
        material=link_gray,
        name="left_ankle_yoke",
    )
    shank_link.visual(
        Box((0.050, 0.016, 0.095)),
        origin=Origin(xyz=(0.000, -0.035, -0.3025)),
        material=link_gray,
        name="right_ankle_yoke",
    )
    shank_link.inertial = Inertial.from_geometry(
        Box((0.110, 0.080, 0.350)),
        mass=4.7,
        origin=Origin(xyz=(0.000, 0.000, -0.175)),
    )

    ankle_foot = model.part("ankle_foot")
    ankle_foot.visual(
        Cylinder(radius=0.028, length=0.055),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="ankle_hub",
    )
    ankle_foot.visual(
        Box((0.070, 0.055, 0.090)),
        origin=Origin(xyz=(0.010, 0.000, -0.045)),
        material=foot_dark,
        name="ankle_block",
    )
    ankle_foot.visual(
        Box((0.200, 0.110, 0.045)),
        origin=Origin(xyz=(0.060, 0.000, -0.095)),
        material=foot_dark,
        name="sole_block",
    )
    ankle_foot.visual(
        Box((0.080, 0.100, 0.025)),
        origin=Origin(xyz=(0.145, 0.000, -0.065)),
        material=rubber,
        name="toe_block",
    )
    ankle_foot.visual(
        Box((0.050, 0.095, 0.035)),
        origin=Origin(xyz=(-0.050, 0.000, -0.095)),
        material=rubber,
        name="heel_pad",
    )
    ankle_foot.inertial = Inertial.from_geometry(
        Box((0.230, 0.120, 0.125)),
        mass=2.6,
        origin=Origin(xyz=(0.050, 0.000, -0.060)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_leg_housing,
        child=thigh_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=950.0,
            velocity=2.0,
            lower=math.radians(-45.0),
            upper=math.radians(65.0),
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.000, 0.000, -0.380)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=2.4,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=ankle_foot,
        origin=Origin(xyz=(0.000, 0.000, -0.350)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=2.6,
            lower=math.radians(-35.0),
            upper=math.radians(35.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("upper_leg_housing")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")

    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.expect_gap(
        housing,
        thigh,
        axis="z",
        positive_elem="hip_bridge",
        negative_elem="thigh_shell",
        min_gap=0.015,
        max_gap=0.080,
        name="thigh shell hangs below the hip bridge",
    )
    ctx.expect_gap(
        thigh,
        shank,
        axis="z",
        positive_elem="knee_bridge",
        negative_elem="shank_shell",
        min_gap=0.030,
        max_gap=0.120,
        name="shank tucks below the knee bridge",
    )
    ctx.expect_gap(
        shank,
        foot,
        axis="z",
        positive_elem="ankle_bridge",
        negative_elem="ankle_block",
        min_gap=0.040,
        max_gap=0.120,
        name="ankle block sits below the shank bridge",
    )
    ctx.expect_overlap(
        thigh,
        shank,
        axes="xy",
        elem_a="thigh_shell",
        elem_b="shank_shell",
        min_overlap=0.040,
        name="thigh and shank stay aligned in the straight pose",
    )
    ctx.expect_overlap(
        shank,
        foot,
        axes="y",
        elem_a="shank_shell",
        elem_b="sole_block",
        min_overlap=0.040,
        name="foot remains centered under the shank",
    )

    rest_knee = ctx.part_world_position(shank)
    rest_ankle = ctx.part_world_position(foot)
    rest_toe = ctx.part_element_world_aabb(foot, elem="toe_block")

    with ctx.pose({hip: math.radians(35.0)}):
        flexed_knee = ctx.part_world_position(shank)
    ctx.check(
        "hip positive rotation swings the knee forward",
        rest_knee is not None
        and flexed_knee is not None
        and flexed_knee[0] > rest_knee[0] + 0.12
        and flexed_knee[2] > rest_knee[2] + 0.04,
        details=f"rest_knee={rest_knee}, flexed_knee={flexed_knee}",
    )

    with ctx.pose({knee: math.radians(90.0)}):
        folded_ankle = ctx.part_world_position(foot)
    ctx.check(
        "knee flex pulls the ankle upward and forward",
        rest_ankle is not None
        and folded_ankle is not None
        and folded_ankle[0] > rest_ankle[0] + 0.18
        and folded_ankle[2] > rest_ankle[2] + 0.10,
        details=f"rest_ankle={rest_ankle}, folded_ankle={folded_ankle}",
    )

    with ctx.pose({ankle: math.radians(25.0)}):
        raised_toe = ctx.part_element_world_aabb(foot, elem="toe_block")
    ctx.check(
        "ankle dorsiflexion raises the toe",
        rest_toe is not None
        and raised_toe is not None
        and raised_toe[1][2] > rest_toe[1][2] + 0.025,
        details=f"rest_toe={rest_toe}, raised_toe={raised_toe}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
