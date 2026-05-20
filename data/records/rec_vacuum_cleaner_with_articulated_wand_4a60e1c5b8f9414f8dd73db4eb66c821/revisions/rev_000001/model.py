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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    body_graphite = model.material("body_graphite", rgba=(0.18, 0.20, 0.23, 1.0))
    body_accent = model.material("body_accent", rgba=(0.78, 0.12, 0.12, 1.0))
    tube_silver = model.material("tube_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.45, 0.55, 0.62, 0.45))

    def yz_section(
        x: float,
        *,
        width: float,
        height: float,
        radius: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_center) for y, z in rounded_rect_profile(width, height, radius)]

    body = model.part("main_body")

    body_shell = section_loft(
        [
            yz_section(-0.105, width=0.118, height=0.085, radius=0.020, z_center=0.052),
            yz_section(-0.030, width=0.162, height=0.132, radius=0.030, z_center=0.068),
            yz_section(0.050, width=0.150, height=0.122, radius=0.028, z_center=0.068),
            yz_section(0.112, width=0.105, height=0.090, radius=0.020, z_center=0.060),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell, "vacuum_body_shell"),
        material=body_graphite,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.094),
        origin=Origin(xyz=(-0.030, 0.0, 0.094), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoked_clear,
        name="dust_bin",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.026),
        origin=Origin(xyz=(0.100, 0.0, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_accent,
        name="body_socket",
    )
    body.visual(
        Box((0.050, 0.106, 0.020)),
        origin=Origin(xyz=(0.000, 0.0, 0.010)),
        material=body_graphite,
        name="lower_skid",
    )
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.040, 0.0, 0.112),
                    (-0.020, 0.0, 0.140),
                    (0.030, 0.0, 0.146),
                    (0.082, 0.0, 0.118),
                ],
                radius=0.010,
                samples_per_segment=18,
                radial_segments=18,
            ),
            "vacuum_top_handle",
        ),
        material=body_accent,
        name="top_handle",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.230, 0.170, 0.150)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.014, length=0.112),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_silver,
        name="upper_wand_tube",
    )
    upper_wand.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_graphite,
        name="upper_wand_cuff",
    )
    upper_wand.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=(0.112, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_graphite,
        name="upper_wand_tip",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.130, 0.040, 0.040)),
        mass=0.28,
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.013, length=0.096),
        origin=Origin(xyz=(0.048, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_silver,
        name="lower_wand_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_graphite,
        name="lower_wand_cuff",
    )
    lower_wand.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.096, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_graphite,
        name="lower_wand_tip",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.112, 0.036, 0.036)),
        mass=0.24,
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
    )

    nozzle = model.part("floor_nozzle")
    nozzle.visual(
        Box((0.020, 0.048, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, -0.004)),
        material=body_graphite,
        name="nozzle_yoke",
    )
    nozzle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.010, 0.0, 0.0),
                    (0.022, 0.0, -0.021),
                    (0.042, 0.0, -0.048),
                ],
                radius=0.008,
                samples_per_segment=12,
                radial_segments=16,
            ),
            "vacuum_nozzle_neck",
        ),
        material=body_graphite,
        name="nozzle_neck",
    )
    nozzle.visual(
        Box((0.112, 0.118, 0.016)),
        origin=Origin(xyz=(0.078, 0.0, -0.067)),
        material=body_graphite,
        name="nozzle_head",
    )
    nozzle.visual(
        Box((0.066, 0.094, 0.016)),
        origin=Origin(xyz=(0.054, 0.0, -0.054)),
        material=body_accent,
        name="nozzle_cover",
    )
    nozzle.visual(
        Box((0.104, 0.108, 0.004)),
        origin=Origin(xyz=(0.080, 0.0, -0.076)),
        material=body_accent,
        name="nozzle_front_lip",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.136, 0.120, 0.082)),
        mass=0.34,
        origin=Origin(xyz=(0.068, 0.0, -0.041)),
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(xyz=(0.113, 0.0, 0.085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.122, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=math.radians(-35.0),
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "lower_wand_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=nozzle,
        origin=Origin(xyz=(0.106, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=math.radians(-28.0),
            upper=math.radians(38.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("main_body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    nozzle = object_model.get_part("floor_nozzle")
    body_to_upper = object_model.get_articulation("body_to_upper_wand")
    upper_to_lower = object_model.get_articulation("upper_to_lower_wand")
    lower_to_nozzle = object_model.get_articulation("lower_wand_to_nozzle")

    ctx.expect_contact(
        upper_wand,
        body,
        elem_a="upper_wand_cuff",
        elem_b="body_socket",
        name="upper wand seats against body socket",
    )
    ctx.expect_contact(
        lower_wand,
        upper_wand,
        elem_a="lower_wand_cuff",
        elem_b="upper_wand_tip",
        name="lower wand seats against upper wand elbow",
    )
    ctx.expect_contact(
        nozzle,
        lower_wand,
        elem_a="nozzle_yoke",
        elem_b="lower_wand_tip",
        name="floor nozzle seats against lower wand hinge",
    )
    ctx.expect_gap(
        lower_wand,
        nozzle,
        axis="z",
        min_gap=0.008,
        positive_elem="lower_wand_tube",
        negative_elem="nozzle_head",
        name="nozzle head hangs below the rigid wand",
    )

    rest_tip_aabb = ctx.part_element_world_aabb(upper_wand, elem="upper_wand_tip")
    with ctx.pose({body_to_upper: math.radians(55.0)}):
        raised_tip_aabb = ctx.part_element_world_aabb(upper_wand, elem="upper_wand_tip")
    ctx.check(
        "upper wand raises with positive elbow motion",
        rest_tip_aabb is not None
        and raised_tip_aabb is not None
        and raised_tip_aabb[1][2] > rest_tip_aabb[1][2] + 0.04,
        details=f"rest={rest_tip_aabb}, raised={raised_tip_aabb}",
    )

    rest_lower_tip_aabb = ctx.part_element_world_aabb(lower_wand, elem="lower_wand_tip")
    with ctx.pose({upper_to_lower: math.radians(60.0)}):
        bent_lower_tip_aabb = ctx.part_element_world_aabb(lower_wand, elem="lower_wand_tip")
    ctx.check(
        "second elbow lifts the distal wand segment",
        rest_lower_tip_aabb is not None
        and bent_lower_tip_aabb is not None
        and bent_lower_tip_aabb[1][2] > rest_lower_tip_aabb[1][2] + 0.04,
        details=f"rest={rest_lower_tip_aabb}, bent={bent_lower_tip_aabb}",
    )

    rest_nozzle_front_aabb = ctx.part_element_world_aabb(nozzle, elem="nozzle_front_lip")
    with ctx.pose({lower_to_nozzle: math.radians(28.0)}):
        pitched_nozzle_front_aabb = ctx.part_element_world_aabb(nozzle, elem="nozzle_front_lip")
    ctx.check(
        "positive nozzle pitch raises the front edge",
        rest_nozzle_front_aabb is not None
        and pitched_nozzle_front_aabb is not None
        and pitched_nozzle_front_aabb[1][2] > rest_nozzle_front_aabb[1][2] + 0.012,
        details=f"rest={rest_nozzle_front_aabb}, pitched={pitched_nozzle_front_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
