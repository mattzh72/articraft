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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="towed_anti_air_launcher")

    nato_green = model.material("nato_green", rgba=(0.31, 0.37, 0.22, 1.0))
    olive_drab = model.material("olive_drab", rgba=(0.40, 0.44, 0.26, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.31, 0.34, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    tube_shell = _mesh(
        "launcher_tube_shell",
        CylinderGeometry(radius=0.10, height=1.85, radial_segments=48, closed=False),
    )

    chassis = model.part("chassis")
    chassis.visual(
        Box((1.70, 0.26, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=nato_green,
        name="spine_beam",
    )
    chassis.visual(
        Box((0.32, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=nato_green,
        name="center_column",
    )
    chassis.visual(
        Box((0.95, 0.52, 0.10)),
        origin=Origin(xyz=(0.05, 0.0, 0.56)),
        material=nato_green,
        name="upper_deck",
    )
    chassis.visual(
        Box((0.42, 0.34, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        material=olive_drab,
        name="yaw_pedestal",
    )
    chassis.visual(
        Cylinder(radius=0.24, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
        material=dark_steel,
        name="yaw_bearing_ring",
    )
    chassis.visual(
        Box((0.95, 0.06, 0.08)),
        origin=Origin(xyz=(0.82, 0.14, 0.36), rpy=(0.0, 0.0, -0.23)),
        material=nato_green,
        name="drawbar_left",
    )
    chassis.visual(
        Box((0.95, 0.06, 0.08)),
        origin=Origin(xyz=(0.82, -0.14, 0.36), rpy=(0.0, 0.0, 0.23)),
        material=nato_green,
        name="drawbar_right",
    )
    chassis.visual(
        Box((0.18, 0.10, 0.10)),
        origin=Origin(xyz=(1.28, 0.0, 0.32)),
        material=dark_steel,
        name="coupler_block",
    )
    chassis.visual(
        Cylinder(radius=0.055, length=0.04),
        origin=Origin(xyz=(1.39, 0.0, 0.32), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="tow_eye",
    )
    chassis.visual(
        Cylinder(radius=0.05, length=1.12),
        origin=Origin(xyz=(-0.22, 0.0, 0.29), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    chassis.visual(
        Box((0.22, 0.10, 0.12)),
        origin=Origin(xyz=(-0.22, 0.18, 0.35)),
        material=dark_steel,
        name="left_spring_block",
    )
    chassis.visual(
        Box((0.22, 0.10, 0.12)),
        origin=Origin(xyz=(-0.22, -0.18, 0.35)),
        material=dark_steel,
        name="right_spring_block",
    )
    chassis.visual(
        Cylinder(radius=0.24, length=0.12),
        origin=Origin(xyz=(-0.22, 0.62, 0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_wheel",
    )
    chassis.visual(
        Cylinder(radius=0.24, length=0.12),
        origin=Origin(xyz=(-0.22, -0.62, 0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_wheel",
    )
    chassis.visual(
        Box((0.12, 0.05, 0.26)),
        origin=Origin(xyz=(-0.44, 0.27, 0.46)),
        material=dark_steel,
        name="left_outrigger_bracket",
    )
    chassis.visual(
        Box((0.12, 0.05, 0.26)),
        origin=Origin(xyz=(-0.44, -0.27, 0.46)),
        material=dark_steel,
        name="right_outrigger_bracket",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((2.05, 1.34, 0.96)),
        mass=1850.0,
        origin=Origin(xyz=(0.20, 0.0, 0.48)),
    )

    launcher_head = model.part("launcher_head")
    launcher_head.visual(
        Cylinder(radius=0.19, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_steel,
        name="slewing_collar",
    )
    launcher_head.visual(
        Box((0.56, 0.62, 0.10)),
        origin=Origin(xyz=(0.02, 0.0, 0.15)),
        material=olive_drab,
        name="rotating_platform",
    )
    launcher_head.visual(
        Box((0.30, 0.34, 0.16)),
        origin=Origin(xyz=(0.08, 0.0, 0.27)),
        material=olive_drab,
        name="center_saddle",
    )
    launcher_head.visual(
        Box((0.20, 0.30, 0.18)),
        origin=Origin(xyz=(-0.20, 0.0, 0.29)),
        material=dark_steel,
        name="azimuth_drive_pack",
    )
    launcher_head.visual(
        Box((0.44, 0.08, 0.44)),
        origin=Origin(xyz=(0.08, 0.32, 0.39)),
        material=olive_drab,
        name="left_yoke_cheek",
    )
    launcher_head.visual(
        Box((0.44, 0.08, 0.44)),
        origin=Origin(xyz=(0.08, -0.32, 0.39)),
        material=olive_drab,
        name="right_yoke_cheek",
    )
    launcher_head.visual(
        Box((0.20, 0.12, 0.18)),
        origin=Origin(xyz=(0.00, 0.26, 0.23)),
        material=dark_steel,
        name="left_yoke_brace",
    )
    launcher_head.visual(
        Box((0.20, 0.12, 0.18)),
        origin=Origin(xyz=(0.00, -0.26, 0.23)),
        material=dark_steel,
        name="right_yoke_brace",
    )
    launcher_head.visual(
        Box((0.20, 0.60, 0.04)),
        origin=Origin(xyz=(-0.08, 0.0, 0.63)),
        material=dark_steel,
        name="top_tie_bar",
    )
    launcher_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.62),
        mass=240.0,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )

    launcher_cradle = model.part("launcher_cradle")
    launcher_cradle.visual(
        Cylinder(radius=0.05, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_shaft",
    )
    launcher_cradle.visual(
        Box((0.24, 0.32, 0.22)),
        origin=Origin(xyz=(-0.12, 0.0, 0.02)),
        material=dark_steel,
        name="rear_mount_block",
    )
    launcher_cradle.visual(
        Box((0.20, 0.52, 0.08)),
        origin=Origin(xyz=(0.06, 0.0, 0.02)),
        material=olive_drab,
        name="tube_crossmember",
    )
    launcher_cradle.visual(
        Box((0.22, 0.12, 0.12)),
        origin=Origin(xyz=(-0.02, 0.18, 0.06)),
        material=olive_drab,
        name="left_tube_shoe",
    )
    launcher_cradle.visual(
        Box((0.22, 0.12, 0.12)),
        origin=Origin(xyz=(-0.02, -0.18, 0.06)),
        material=olive_drab,
        name="right_tube_shoe",
    )
    launcher_cradle.visual(
        Box((0.16, 0.10, 0.18)),
        origin=Origin(xyz=(-0.03, 0.18, 0.09)),
        material=dark_steel,
        name="left_trunnion_web",
    )
    launcher_cradle.visual(
        Box((0.16, 0.10, 0.18)),
        origin=Origin(xyz=(-0.03, -0.18, 0.09)),
        material=dark_steel,
        name="right_trunnion_web",
    )
    launcher_cradle.visual(
        tube_shell,
        origin=Origin(xyz=(0.82, 0.18, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nato_green,
        name="left_tube",
    )
    launcher_cradle.visual(
        tube_shell,
        origin=Origin(xyz=(0.82, -0.18, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nato_green,
        name="right_tube",
    )
    launcher_cradle.visual(
        Box((0.08, 0.08, 0.22)),
        origin=Origin(xyz=(0.34, 0.0, 0.17)),
        material=dark_steel,
        name="optic_mast",
    )
    launcher_cradle.visual(
        Box((0.18, 0.12, 0.12)),
        origin=Origin(xyz=(0.40, 0.0, 0.28)),
        material=dark_steel,
        name="sight_box",
    )
    launcher_cradle.visual(
        Box((0.30, 0.08, 0.05)),
        origin=Origin(xyz=(0.25, 0.0, 0.085)),
        material=dark_steel,
        name="sight_support_bridge",
    )
    launcher_cradle.inertial = Inertial.from_geometry(
        Box((2.00, 0.72, 0.44)),
        mass=165.0,
        origin=Origin(xyz=(0.72, 0.0, 0.12)),
    )

    left_outrigger = model.part("left_outrigger")
    left_outrigger.visual(
        Cylinder(radius=0.015, length=0.065),
        material=gunmetal,
        name="left_hinge_sleeve",
    )
    left_outrigger.visual(
        Box((0.56, 0.08, 0.08)),
        origin=Origin(xyz=(-0.28, 0.04, 0.0)),
        material=olive_drab,
        name="left_stabilizer_beam",
    )
    left_outrigger.visual(
        Box((0.08, 0.08, 0.24)),
        origin=Origin(xyz=(-0.52, 0.09, -0.16)),
        material=olive_drab,
        name="left_support_leg",
    )
    left_outrigger.visual(
        Box((0.14, 0.14, 0.03)),
        origin=Origin(xyz=(-0.52, 0.09, -0.295)),
        material=dark_steel,
        name="left_footpad",
    )
    left_outrigger.visual(
        Box((0.30, 0.05, 0.05)),
        origin=Origin(xyz=(-0.34, 0.08, -0.08), rpy=(0.0, 0.34, 0.0)),
        material=dark_steel,
        name="left_diagonal_brace",
    )
    left_outrigger.inertial = Inertial.from_geometry(
        Box((0.62, 0.18, 0.34)),
        mass=40.0,
        origin=Origin(xyz=(-0.30, 0.08, -0.14)),
    )

    right_outrigger = model.part("right_outrigger")
    right_outrigger.visual(
        Cylinder(radius=0.015, length=0.065),
        material=gunmetal,
        name="right_hinge_sleeve",
    )
    right_outrigger.visual(
        Box((0.56, 0.08, 0.08)),
        origin=Origin(xyz=(-0.28, -0.04, 0.0)),
        material=olive_drab,
        name="right_stabilizer_beam",
    )
    right_outrigger.visual(
        Box((0.08, 0.08, 0.24)),
        origin=Origin(xyz=(-0.52, -0.09, -0.16)),
        material=olive_drab,
        name="right_support_leg",
    )
    right_outrigger.visual(
        Box((0.14, 0.14, 0.03)),
        origin=Origin(xyz=(-0.52, -0.09, -0.295)),
        material=dark_steel,
        name="right_footpad",
    )
    right_outrigger.visual(
        Box((0.30, 0.05, 0.05)),
        origin=Origin(xyz=(-0.34, -0.08, -0.08), rpy=(0.0, 0.34, 0.0)),
        material=dark_steel,
        name="right_diagonal_brace",
    )
    right_outrigger.inertial = Inertial.from_geometry(
        Box((0.62, 0.18, 0.34)),
        mass=40.0,
        origin=Origin(xyz=(-0.30, -0.08, -0.14)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=launcher_head,
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.8),
    )
    model.articulation(
        "elevation_rotation",
        ArticulationType.REVOLUTE,
        parent=launcher_head,
        child=launcher_cradle,
        origin=Origin(xyz=(0.14, 0.0, 0.44)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.7,
            lower=math.radians(-8.0),
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "left_outrigger_fold",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=left_outrigger,
        origin=Origin(xyz=(-0.48, 0.31, 0.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=1.0,
            lower=math.radians(-82.0),
            upper=math.radians(4.0),
        ),
    )
    model.articulation(
        "right_outrigger_fold",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=right_outrigger,
        origin=Origin(xyz=(-0.48, -0.31, 0.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=1.0,
            lower=math.radians(-4.0),
            upper=math.radians(82.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    launcher_head = object_model.get_part("launcher_head")
    launcher_cradle = object_model.get_part("launcher_cradle")
    left_outrigger = object_model.get_part("left_outrigger")
    right_outrigger = object_model.get_part("right_outrigger")

    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_rotation")
    left_fold = object_model.get_articulation("left_outrigger_fold")
    right_fold = object_model.get_articulation("right_outrigger_fold")

    def _span(aabb, axis_index: int) -> float:
        return aabb[1][axis_index] - aabb[0][axis_index]

    def _center(aabb, axis_index: int) -> float:
        return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "azimuth joint axis is vertical",
        azimuth.axis == (0.0, 0.0, 1.0),
        f"expected vertical azimuth axis, got {azimuth.axis}",
    )
    ctx.check(
        "elevation joint axis is transverse",
        elevation.axis == (0.0, -1.0, 0.0),
        f"expected horizontal elevation axis, got {elevation.axis}",
    )
    ctx.check(
        "outrigger hinges fold on vertical pins",
        left_fold.axis == (0.0, 0.0, 1.0) and right_fold.axis == (0.0, 0.0, 1.0),
        f"left={left_fold.axis}, right={right_fold.axis}",
    )

    ctx.expect_contact(launcher_head, chassis, name="launcher head sits on yaw bearing")
    ctx.expect_contact(launcher_cradle, launcher_head, name="cradle is seated in yoke")
    ctx.expect_contact(left_outrigger, chassis, name="left outrigger remains clipped to chassis hinge")
    ctx.expect_contact(right_outrigger, chassis, name="right outrigger remains clipped to chassis hinge")

    left_tube_rest = ctx.part_element_world_aabb(launcher_cradle, elem="left_tube")
    assert left_tube_rest is not None
    left_out_rest = ctx.part_world_aabb(left_outrigger)
    assert left_out_rest is not None
    right_out_rest = ctx.part_world_aabb(right_outrigger)
    assert right_out_rest is not None

    with ctx.pose({azimuth: math.pi / 2.0}):
        left_tube_turned = ctx.part_element_world_aabb(launcher_cradle, elem="left_tube")
        assert left_tube_turned is not None
        ctx.check(
            "azimuth rotation swings tube footprint around yaw axis",
            _span(left_tube_rest, 0) > 1.5
            and _span(left_tube_turned, 1) > 1.5
            and abs(_center(left_tube_turned, 1)) > 0.5,
            f"rest={left_tube_rest}, turned={left_tube_turned}",
        )
        ctx.expect_contact(launcher_head, chassis)

    with ctx.pose({elevation: math.radians(55.0)}):
        left_tube_raised = ctx.part_element_world_aabb(launcher_cradle, elem="left_tube")
        assert left_tube_raised is not None
        ctx.check(
            "elevation raises the launch tubes",
            left_tube_raised[1][2] > left_tube_rest[1][2] + 0.85,
            f"rest={left_tube_rest}, raised={left_tube_raised}",
        )
        ctx.expect_contact(launcher_cradle, launcher_head)

    with ctx.pose({left_fold: math.radians(-78.0)}):
        left_out_deployed = ctx.part_world_aabb(left_outrigger)
        assert left_out_deployed is not None
        ctx.check(
            "left outrigger swings outward from the chassis side",
            left_out_deployed[1][1] > left_out_rest[1][1] + 0.32,
            f"rest={left_out_rest}, deployed={left_out_deployed}",
        )
        ctx.expect_contact(left_outrigger, chassis)
        ctx.expect_gap(
            left_outrigger,
            chassis,
            axis="y",
            min_gap=0.40,
            positive_elem="left_footpad",
            negative_elem="upper_deck",
            name="left outrigger clears side when deployed",
        )

    with ctx.pose({right_fold: math.radians(78.0)}):
        right_out_deployed = ctx.part_world_aabb(right_outrigger)
        assert right_out_deployed is not None
        ctx.check(
            "right outrigger swings outward from the chassis side",
            right_out_deployed[0][1] < right_out_rest[0][1] - 0.32,
            f"rest={right_out_rest}, deployed={right_out_deployed}",
        )
        ctx.expect_contact(right_outrigger, chassis)
        ctx.expect_gap(
            chassis,
            right_outrigger,
            axis="y",
            min_gap=0.40,
            positive_elem="upper_deck",
            negative_elem="right_footpad",
            name="right outrigger clears side when deployed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
