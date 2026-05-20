from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


SHELL_WIDTH = 0.92
SHELL_DEPTH = 0.235
SHELL_HEIGHT = 0.285
OUTLET_WIDTH = 0.884


def _shell_section(
    y_pos: float,
    *,
    width: float,
    z_min: float,
    z_max: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(width, z_max - z_min, radius, corner_segments=10)
    z_center = (z_min + z_max) * 0.5
    return [(x, y_pos, z_center + z) for x, z in profile]


def _build_housing_shell():
    return section_loft(
        [
            _shell_section(
                -0.112,
                width=SHELL_WIDTH,
                z_min=0.020,
                z_max=0.278,
                radius=0.048,
            ),
            _shell_section(
                -0.060,
                width=SHELL_WIDTH,
                z_min=0.014,
                z_max=SHELL_HEIGHT,
                radius=0.054,
            ),
            _shell_section(
                0.000,
                width=0.914,
                z_min=0.026,
                z_max=0.279,
                radius=0.058,
            ),
            _shell_section(
                0.056,
                width=0.904,
                z_min=0.060,
                z_max=0.266,
                radius=0.048,
            ),
            _shell_section(
                0.104,
                width=0.892,
                z_min=0.112,
                z_max=0.226,
                radius=0.030,
            ),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_split_air_conditioner")

    shell_white = model.material("shell_white", rgba=(0.93, 0.95, 0.96, 1.0))
    shell_shadow = model.material("shell_shadow", rgba=(0.24, 0.28, 0.30, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_build_housing_shell(), "mini_split_housing_shell"),
        material=shell_white,
        name="shell",
    )
    housing.visual(
        Box((SHELL_WIDTH, 0.011, 0.235)),
        origin=Origin(xyz=(0.0, -0.1175, 0.1275)),
        material=shell_shadow,
        name="wall_plate",
    )
    housing.visual(
        Box((0.900, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, 0.101, 0.095)),
        material=shell_white,
        name="lower_lip",
    )

    flap = model.part("flap")
    flap.visual(
        Box((OUTLET_WIDTH, 0.046, 0.005)),
        origin=Origin(xyz=(0.0, 0.051, -0.0065)),
        material=shell_white,
        name="flap_blade",
    )
    flap.visual(
        Box((OUTLET_WIDTH, 0.004, 0.007)),
        origin=Origin(xyz=(0.0, 0.032, -0.0055)),
        material=shell_shadow,
        name="flap_spine",
    )
    flap_hinge = model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(0.0, 0.087, 0.109)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.6,
            lower=0.0,
            upper=1.20,
        ),
    )
    flap_hinge.meta["qc_samples"] = [0.0, 0.55, 1.10]

    control_cluster = model.part("control_cluster")
    control_cluster.visual(
        Box((0.001, 0.110, 0.138)),
        origin=Origin(xyz=(0.0005, 0.0, 0.0)),
        material=shell_shadow,
        name="cluster_panel",
    )
    control_cluster.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(
            xyz=(0.005, 0.0, 0.033),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=shell_shadow,
        name="dial_pod",
    )
    control_cluster.visual(
        Box((0.004, 0.004, 0.054)),
        origin=Origin(xyz=(0.002, -0.021, -0.040)),
        material=shell_shadow,
        name="button_rail_inner",
    )
    control_cluster.visual(
        Box((0.004, 0.004, 0.054)),
        origin=Origin(xyz=(0.002, 0.021, -0.040)),
        material=shell_shadow,
        name="button_rail_outer",
    )
    control_cluster.visual(
        Box((0.004, 0.042, 0.004)),
        origin=Origin(xyz=(0.002, 0.0, -0.018)),
        material=shell_shadow,
        name="button_rail_top",
    )
    control_cluster.visual(
        Box((0.004, 0.042, 0.003)),
        origin=Origin(xyz=(0.002, 0.0, -0.040)),
        material=shell_shadow,
        name="button_divider",
    )
    control_cluster.visual(
        Box((0.004, 0.042, 0.004)),
        origin=Origin(xyz=(0.002, 0.0, -0.062)),
        material=shell_shadow,
        name="button_rail_bottom",
    )
    model.articulation(
        "housing_to_control_cluster",
        ArticulationType.FIXED,
        parent=housing,
        child=control_cluster,
        origin=Origin(xyz=(0.457, 0.008, 0.152)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(
            xyz=(0.004, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=shell_shadow,
        name="dial_shaft",
    )
    dial.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(
            xyz=(0.015, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=shell_white,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(
            xyz=(0.009, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=shell_white,
        name="dial_skirt",
    )
    dial.visual(
        Box((0.003, 0.006, 0.004)),
        origin=Origin(xyz=(0.0235, 0.0, 0.012)),
        material=shell_shadow,
        name="dial_pointer",
    )
    dial_joint = model.articulation(
        "control_cluster_to_dial",
        ArticulationType.CONTINUOUS,
        parent=control_cluster,
        child=dial,
        origin=Origin(xyz=(0.010, 0.0, 0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=14.0),
    )
    dial_joint.meta["qc_samples"] = [0.0, 1.2, 2.4]

    upper_button = model.part("upper_button")
    upper_button.visual(
        Box((0.004, 0.034, 0.014)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=shell_white,
        name="button_cap",
    )
    upper_button.visual(
        Box((0.003, 0.012, 0.006)),
        origin=Origin(xyz=(0.0025, 0.0, 0.0)),
        material=shell_shadow,
        name="button_stem",
    )
    upper_button.visual(
        Box((0.002, 0.024, 0.008)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=shell_shadow,
        name="button_face",
    )
    upper_button_joint = model.articulation(
        "control_cluster_to_upper_button",
        ArticulationType.PRISMATIC,
        parent=control_cluster,
        child=upper_button,
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.002,
        ),
    )
    upper_button_joint.meta["qc_samples"] = [0.0, 0.002]

    lower_button = model.part("lower_button")
    lower_button.visual(
        Box((0.004, 0.034, 0.014)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=shell_white,
        name="button_cap",
    )
    lower_button.visual(
        Box((0.003, 0.012, 0.006)),
        origin=Origin(xyz=(0.0025, 0.0, 0.0)),
        material=shell_shadow,
        name="button_stem",
    )
    lower_button.visual(
        Box((0.002, 0.024, 0.008)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=shell_shadow,
        name="button_face",
    )
    lower_button_joint = model.articulation(
        "control_cluster_to_lower_button",
        ArticulationType.PRISMATIC,
        parent=control_cluster,
        child=lower_button,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.002,
        ),
    )
    lower_button_joint.meta["qc_samples"] = [0.0, 0.002]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    flap_hinge = object_model.get_articulation("housing_to_flap")
    dial = object_model.get_part("dial")
    upper_button = object_model.get_part("upper_button")
    lower_button = object_model.get_part("lower_button")
    dial_joint = object_model.get_articulation("control_cluster_to_dial")
    upper_button_joint = object_model.get_articulation("control_cluster_to_upper_button")
    lower_button_joint = object_model.get_articulation("control_cluster_to_lower_button")

    ctx.expect_overlap(
        flap,
        housing,
        axes="x",
        elem_a="flap_blade",
        elem_b="lower_lip",
        min_overlap=0.84,
        name="flap spans nearly the full outlet width",
    )
    ctx.expect_gap(
        flap,
        housing,
        axis="y",
        positive_elem="flap_blade",
        negative_elem="lower_lip",
        min_gap=-0.001,
        max_gap=0.004,
        name="closed flap nests just ahead of the discharge lip",
    )

    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 1.10}):
        open_aabb = ctx.part_world_aabb(flap)

    ctx.check(
        "flap opens downward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < rest_aabb[0][2] - 0.025
        and open_aabb[0][1] < rest_aabb[0][1] - 0.006,
        details=f"rest={rest_aabb!r}, open={open_aabb!r}",
    )

    dial_pointer_rest = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_joint: 1.2}):
        dial_pointer_turn = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    ctx.check(
        "dial rotates about its shaft",
        dial_pointer_rest is not None
        and dial_pointer_turn is not None
        and (
            abs(dial_pointer_turn[0][1] - dial_pointer_rest[0][1]) > 0.006
            or abs(dial_pointer_turn[1][2] - dial_pointer_rest[1][2]) > 0.006
        ),
        details=f"rest={dial_pointer_rest!r}, turn={dial_pointer_turn!r}",
    )

    upper_rest = ctx.part_world_position(upper_button)
    lower_rest = ctx.part_world_position(lower_button)
    with ctx.pose({upper_button_joint: 0.002}):
        upper_pressed = ctx.part_world_position(upper_button)
        lower_during_upper = ctx.part_world_position(lower_button)
    with ctx.pose({lower_button_joint: 0.002}):
        lower_pressed = ctx.part_world_position(lower_button)
        upper_during_lower = ctx.part_world_position(upper_button)

    ctx.check(
        "upper button depresses independently",
        upper_rest is not None
        and lower_rest is not None
        and upper_pressed is not None
        and lower_during_upper is not None
        and upper_pressed[0] < upper_rest[0] - 0.0015
        and abs(lower_during_upper[0] - lower_rest[0]) < 0.0005,
        details=(
            f"upper_rest={upper_rest!r}, upper_pressed={upper_pressed!r}, "
            f"lower_rest={lower_rest!r}, lower_during_upper={lower_during_upper!r}"
        ),
    )
    ctx.check(
        "lower button depresses independently",
        upper_rest is not None
        and lower_rest is not None
        and lower_pressed is not None
        and upper_during_lower is not None
        and lower_pressed[0] < lower_rest[0] - 0.0015
        and abs(upper_during_lower[0] - upper_rest[0]) < 0.0005,
        details=(
            f"lower_rest={lower_rest!r}, lower_pressed={lower_pressed!r}, "
            f"upper_rest={upper_rest!r}, upper_during_lower={upper_during_lower!r}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
