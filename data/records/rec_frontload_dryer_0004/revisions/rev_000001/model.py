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
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 64, center: tuple[float, float] = (0.0, 0.0)) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _rect_profile(width: float, height: float, *, center: tuple[float, float] = (0.0, 0.0)) -> list[tuple[float, float]]:
    cx, cy = center
    hx = width / 2.0
    hy = height / 2.0
    return [
        (cx - hx, cy - hy),
        (cx + hx, cy - hy),
        (cx + hx, cy + hy),
        (cx - hx, cy + hy),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacked_dryer_unit")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.76, 0.79, 0.82, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.50, 0.63, 0.72, 0.35))

    body_width = 0.68
    body_depth = 0.72
    body_height = 1.83
    wall = 0.02
    lower_section_top = 0.82
    seam_thickness = 0.025
    upper_front_height = body_height - wall - (lower_section_top + seam_thickness)
    upper_front_center_z = lower_section_top + seam_thickness + upper_front_height / 2.0
    door_center_z = 1.26
    front_y = (body_depth - wall) / 2.0
    back_y = -front_y

    opening_radius = 0.255
    door_outer_radius = 0.30
    door_glass_radius = 0.208
    door_thickness = 0.045

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(-(body_width - wall) / 2.0, 0.0, body_height / 2.0)),
        material=body_white,
        name="left_side",
    )
    cabinet.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=((body_width - wall) / 2.0, 0.0, body_height / 2.0)),
        material=body_white,
        name="right_side",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall, body_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=body_white,
        name="base_panel",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall, body_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_height - wall / 2.0)),
        material=body_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall, wall, body_height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, back_y, body_height / 2.0)),
        material=body_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall, body_depth, seam_thickness)),
        origin=Origin(xyz=(0.0, 0.0, lower_section_top + seam_thickness / 2.0)),
        material=body_white,
        name="stack_seam",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall, wall, lower_section_top - wall)),
        origin=Origin(xyz=(0.0, front_y, (lower_section_top - wall) / 2.0 + wall)),
        material=body_white,
        name="lower_front_panel",
    )

    upper_front_geom = ExtrudeWithHolesGeometry(
        _rect_profile(body_width - 2.0 * wall, upper_front_height),
        [_circle_profile(opening_radius, center=(0.0, door_center_z - upper_front_center_z))],
        wall,
        cap=True,
        center=True,
    )
    cabinet.visual(
        mesh_from_geometry(upper_front_geom, "upper_front_fascia"),
        origin=Origin(xyz=(0.0, front_y, upper_front_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_white,
        name="upper_front_fascia",
    )

    gasket_geom = ExtrudeWithHolesGeometry(
        _circle_profile(opening_radius + 0.02),
        [_circle_profile(opening_radius - 0.02)],
        0.018,
        cap=True,
        center=True,
    )
    cabinet.visual(
        mesh_from_geometry(gasket_geom, "opening_gasket"),
        origin=Origin(xyz=(0.0, 0.337, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="opening_gasket",
    )
    cabinet.visual(
        Box((0.48, 0.01, 0.10)),
        origin=Origin(xyz=(0.0, 0.355, 1.69)),
        material=panel_dark,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.028, 0.08, 0.62)),
        origin=Origin(xyz=(0.329, 0.375, door_center_z)),
        material=trim_gray,
        name="door_hinge_block",
    )
    cabinet.visual(
        Cylinder(radius=0.032, length=0.04),
        origin=Origin(xyz=(0.0, -0.32, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="rear_bearing",
    )

    drum = model.part("drum")
    drum_outer_profile = [
        (0.262, -0.260),
        (0.268, -0.180),
        (0.270, 0.000),
        (0.268, 0.180),
        (0.262, 0.260),
    ]
    drum_inner_profile = [
        (0.247, -0.255),
        (0.245, 0.000),
        (0.247, 0.255),
    ]
    drum_shell_geom = LatheGeometry.from_shell_profiles(
        drum_outer_profile,
        drum_inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    drum.visual(
        mesh_from_geometry(drum_shell_geom, "drum_shell"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.249, length=0.018),
        origin=Origin(xyz=(0.0, -0.251, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="rear_bulkhead",
    )
    drum.visual(
        Cylinder(radius=0.030, length=0.112),
        origin=Origin(xyz=(0.0, -0.299, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="rear_axle",
    )
    drum.visual(
        Box((0.08, 0.42, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.233)),
        material=trim_gray,
        name="drum_baffle",
    )

    door = model.part("door")
    door_ring_geom = ExtrudeWithHolesGeometry(
        _circle_profile(door_outer_radius),
        [_circle_profile(0.205)],
        door_thickness,
        cap=True,
        center=True,
    )
    door.visual(
        mesh_from_geometry(door_ring_geom, "door_ring"),
        origin=Origin(xyz=(-door_outer_radius, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_white,
        name="door_ring",
    )
    door_glass_geom = LatheGeometry(
        [
            (0.0, -0.006),
            (0.190, -0.006),
            (0.208, -0.001),
            (0.204, 0.010),
            (0.165, 0.020),
            (0.0, 0.020),
        ],
        segments=56,
    )
    door.visual(
        mesh_from_geometry(door_glass_geom, "door_glass"),
        origin=Origin(xyz=(-door_outer_radius, 0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass_smoke,
        name="door_glass",
    )
    door.visual(
        Box((0.014, 0.05, 0.62)),
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
        material=trim_gray,
        name="hinge_leaf",
    )
    door.visual(
        Box((0.03, 0.03, 0.17)),
        origin=Origin(xyz=(-0.54, 0.015, 0.0)),
        material=trim_gray,
        name="door_handle",
    )

    model.articulation(
        "drum_spin",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, 0.055, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=10.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.315, 0.39, door_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_spin = object_model.get_articulation("drum_spin")
    door_hinge = object_model.get_articulation("door_hinge")

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
        "drum_spin_axis_is_front_to_back",
        drum_spin.axis == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {drum_spin.axis}",
    )
    ctx.check(
        "door_hinge_axis_is_right_edge_vertical",
        door_hinge.axis == (0.0, 0.0, -1.0),
        details=f"expected (0, 0, -1), got {door_hinge.axis}",
    )

    ctx.expect_contact(drum, cabinet, elem_a="rear_axle", elem_b="rear_bearing")
    ctx.expect_contact(door, cabinet, elem_a="hinge_leaf", elem_b="door_hinge_block")
    ctx.expect_within(drum, cabinet, axes="xz", margin=0.0)

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            elem_a="door_ring",
            elem_b="upper_front_fascia",
            min_overlap=0.45,
        )
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="door_ring",
            negative_elem="upper_front_fascia",
            min_gap=0.005,
            max_gap=0.02,
        )
        ctx.expect_gap(
            door,
            drum,
            axis="y",
            positive_elem="door_glass",
            negative_elem="drum_shell",
            min_gap=0.02,
            max_gap=0.08,
        )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    assert closed_door_aabb is not None
    with ctx.pose({door_hinge: math.radians(95.0)}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
        assert open_door_aabb is not None
        assert open_door_aabb[0][0] > closed_door_aabb[0][0] + 0.35
        assert open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.25

    rest_baffle_aabb = ctx.part_element_world_aabb(drum, elem="drum_baffle")
    assert rest_baffle_aabb is not None
    with ctx.pose({drum_spin: math.pi / 2.0}):
        spun_baffle_aabb = ctx.part_element_world_aabb(drum, elem="drum_baffle")
        assert spun_baffle_aabb is not None
        assert spun_baffle_aabb[0][0] > rest_baffle_aabb[1][0] + 0.10
        ctx.expect_contact(drum, cabinet, elem_a="rear_axle", elem_b="rear_bearing")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
