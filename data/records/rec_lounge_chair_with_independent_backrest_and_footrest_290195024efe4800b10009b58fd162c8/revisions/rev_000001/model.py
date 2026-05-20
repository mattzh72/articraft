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
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recovery_lounge_chair")

    shell_polymer = model.material("shell_polymer", rgba=(0.78, 0.79, 0.80, 1.0))
    upholstery = model.material("upholstery", rgba=(0.34, 0.42, 0.48, 1.0))
    frame_metal = model.material("frame_metal", rgba=(0.19, 0.20, 0.22, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.13, 0.14, 1.0))

    def _yz_panel_sections(
        *,
        width: float,
        height: float,
        thickness: float,
        corner_radius: float,
        center_bulge: float = 0.0,
    ) -> MeshGeometry:
        front = [
            (-0.5 * thickness, y, z)
            for y, z in rounded_rect_profile(width, height, corner_radius)
        ]
        mid = [
            (0.0, y, z)
            for y, z in rounded_rect_profile(
                width + center_bulge,
                height + center_bulge * 0.55,
                corner_radius,
            )
        ]
        rear = [
            (0.5 * thickness, y, z)
            for y, z in rounded_rect_profile(width, height, corner_radius)
        ]
        return section_loft([front, mid, rear])

    shell_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.54, 0.58, 0.055),
            0.012,
        ),
        "seat_shell_plate",
    )
    back_panel_mesh = mesh_from_geometry(
        _yz_panel_sections(
            width=0.34,
            height=0.72,
            thickness=0.028,
            corner_radius=0.050,
            center_bulge=0.020,
        ),
        "backrest_panel",
    )
    leg_panel_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.72, 0.30, 0.032),
            0.018,
        ),
        "leg_rest_panel",
    )

    left_frame_geom = wire_from_points(
        [
            (-0.31, 0.295, 0.018),
            (-0.22, 0.295, 0.110),
            (-0.08, 0.295, 0.310),
            (0.04, 0.295, 0.335),
            (0.12, 0.295, 0.332),
            (0.24, 0.295, 0.185),
            (0.31, 0.295, 0.040),
            (0.35, 0.295, 0.018),
        ],
        radius=0.016,
        closed_path=False,
        cap_ends=True,
        corner_mode="miter",
    )
    right_frame_geom = left_frame_geom.copy().scale(1.0, -1.0, 1.0)
    left_frame_mesh = mesh_from_geometry(left_frame_geom, "left_lounge_frame")
    right_frame_mesh = mesh_from_geometry(right_frame_geom, "right_lounge_frame")
    seat_shell = model.part("seat_shell")
    seat_shell.visual(
        shell_plate_mesh,
        origin=Origin(xyz=(0.015, 0.0, 0.438)),
        material=shell_polymer,
        name="shell_pan",
    )
    seat_shell.visual(
        Box((0.036, 0.580, 0.100)),
        origin=Origin(xyz=(0.248, 0.0, 0.392)),
        material=shell_polymer,
        name="front_lip",
    )
    seat_shell.visual(
        Box((0.060, 0.420, 0.120)),
        origin=Origin(xyz=(-0.248, 0.0, 0.416)),
        material=shell_polymer,
        name="rear_web",
    )
    seat_shell.visual(
        Box((0.500, 0.018, 0.122)),
        origin=Origin(xyz=(0.000, 0.281, 0.448)),
        material=shell_polymer,
        name="left_shell_wall",
    )
    seat_shell.visual(
        Box((0.500, 0.018, 0.122)),
        origin=Origin(xyz=(0.000, -0.281, 0.448)),
        material=shell_polymer,
        name="right_shell_wall",
    )
    seat_shell.visual(
        Box((0.090, 0.420, 0.050)),
        origin=Origin(xyz=(-0.240, 0.0, 0.501)),
        material=shell_polymer,
        name="rear_bridge",
    )
    seat_shell.visual(
        Box((0.016, 0.036, 0.090)),
        origin=Origin(xyz=(-0.290, 0.205, 0.548)),
        material=trim_dark,
        name="left_hinge_lug",
    )
    seat_shell.visual(
        Box((0.016, 0.036, 0.090)),
        origin=Origin(xyz=(-0.290, -0.205, 0.548)),
        material=trim_dark,
        name="right_hinge_lug",
    )
    seat_shell.visual(
        Box((0.020, 0.040, 0.070)),
        origin=Origin(xyz=(0.268, 0.165, 0.328)),
        material=trim_dark,
        name="left_leg_bracket",
    )
    seat_shell.visual(
        Box((0.020, 0.040, 0.070)),
        origin=Origin(xyz=(0.268, -0.165, 0.328)),
        material=trim_dark,
        name="right_leg_bracket",
    )
    seat_shell.visual(
        Box((0.440, 0.200, 0.050)),
        origin=Origin(xyz=(0.020, 0.0, 0.382)),
        material=trim_dark,
        name="underslung_beam",
    )
    seat_shell.visual(
        left_frame_mesh,
        material=frame_metal,
        name="left_frame",
    )
    seat_shell.visual(
        right_frame_mesh,
        material=frame_metal,
        name="right_frame",
    )
    seat_shell.visual(
        Cylinder(radius=0.014, length=0.560),
        origin=Origin(xyz=(-0.220, 0.0, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="rear_stretcher",
    )
    seat_shell.visual(
        Cylinder(radius=0.014, length=0.590),
        origin=Origin(xyz=(0.310, 0.0, 0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="front_stretcher",
    )
    seat_shell.visual(
        Cylinder(radius=0.014, length=0.590),
        origin=Origin(xyz=(0.040, 0.0, 0.335), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="seat_support_bar",
    )
    seat_shell.visual(
        Box((0.060, 0.022, 0.120)),
        origin=Origin(xyz=(0.040, 0.281, 0.385)),
        material=trim_dark,
        name="left_shell_mount",
    )
    seat_shell.visual(
        Box((0.060, 0.022, 0.120)),
        origin=Origin(xyz=(0.040, -0.281, 0.385)),
        material=trim_dark,
        name="right_shell_mount",
    )
    seat_shell.inertial = Inertial.from_geometry(
        Box((0.720, 0.620, 0.650)),
        mass=14.0,
        origin=Origin(xyz=(0.020, 0.0, 0.325)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        back_panel_mesh,
        origin=Origin(xyz=(-0.038, 0.0, 0.372), rpy=(0.0, -0.14, 0.0)),
        material=upholstery,
        name="back_panel",
    )
    backrest.visual(
        Box((0.030, 0.260, 0.084)),
        origin=Origin(xyz=(-0.020, 0.0, 0.080), rpy=(0.0, -0.08, 0.0)),
        material=trim_dark,
        name="back_spine",
    )
    backrest.visual(
        Cylinder(radius=0.012, length=0.034),
        origin=Origin(xyz=(0.0, 0.170, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="left_back_hinge_cheek",
    )
    backrest.visual(
        Cylinder(radius=0.012, length=0.034),
        origin=Origin(xyz=(0.0, -0.170, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="right_back_hinge_cheek",
    )
    backrest.visual(
        Box((0.026, 0.374, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=trim_dark,
        name="back_hinge_crossbar",
    )
    backrest.visual(
        Box((0.028, 0.240, 0.120)),
        origin=Origin(xyz=(-0.006, 0.0, 0.072)),
        material=trim_dark,
        name="back_lower_spine",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.090, 0.420, 0.780)),
        mass=2.6,
        origin=Origin(xyz=(-0.020, 0.0, 0.360)),
    )

    leg_rest = model.part("leg_rest")
    leg_rest.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.000, 0.205, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="left_leg_hinge_block",
    )
    leg_rest.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.000, -0.205, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="right_leg_hinge_block",
    )
    leg_rest.visual(
        Box((0.020, 0.080, 0.018)),
        origin=Origin(xyz=(0.288, 0.174, 0.006)),
        material=trim_dark,
        name="left_arm_knuckle",
    )
    leg_rest.visual(
        Box((0.020, 0.080, 0.018)),
        origin=Origin(xyz=(0.288, -0.174, 0.006)),
        material=trim_dark,
        name="right_arm_knuckle",
    )
    leg_rest.visual(
        Box((0.430, 0.018, 0.024)),
        origin=Origin(xyz=(0.505, 0.143, 0.012)),
        material=frame_metal,
        name="left_support_arm",
    )
    leg_rest.visual(
        Box((0.430, 0.018, 0.024)),
        origin=Origin(xyz=(0.505, -0.143, 0.012)),
        material=frame_metal,
        name="right_support_arm",
    )
    leg_rest.visual(
        leg_panel_mesh,
        origin=Origin(xyz=(0.460, 0.0, 0.020)),
        material=upholstery,
        name="leg_panel",
    )
    leg_rest.visual(
        Box((0.500, 0.240, 0.012)),
        origin=Origin(xyz=(0.420, 0.0, 0.014)),
        material=trim_dark,
        name="panel_undertray",
    )
    leg_rest.visual(
        Box((0.420, 0.012, 0.036)),
        origin=Origin(xyz=(0.340, 0.135, 0.016)),
        material=trim_dark,
        name="left_clip_rail",
    )
    leg_rest.visual(
        Box((0.420, 0.012, 0.036)),
        origin=Origin(xyz=(0.340, -0.135, 0.016)),
        material=trim_dark,
        name="right_clip_rail",
    )
    leg_rest.visual(
        Cylinder(radius=0.012, length=0.300),
        origin=Origin(xyz=(0.720, 0.0, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="front_panel_tube",
    )
    leg_rest.inertial = Inertial.from_geometry(
        Box((0.840, 0.320, 0.100)),
        mass=2.2,
        origin=Origin(xyz=(0.420, 0.0, 0.025)),
    )

    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=backrest,
        origin=Origin(xyz=(-0.270, 0.0, 0.548)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.20,
            upper=0.70,
        ),
    )
    model.articulation(
        "seat_to_leg_rest",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=leg_rest,
        origin=Origin(xyz=(0.270, 0.0, 0.330)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.75,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    seat_shell = object_model.get_part("seat_shell")
    backrest = object_model.get_part("backrest")
    leg_rest = object_model.get_part("leg_rest")
    back_hinge = object_model.get_articulation("seat_to_backrest")
    leg_hinge = object_model.get_articulation("seat_to_leg_rest")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(backrest, seat_shell)
    ctx.expect_contact(leg_rest, seat_shell)
    ctx.expect_origin_gap(backrest, seat_shell, axis="z", min_gap=0.12)
    ctx.expect_origin_gap(leg_rest, seat_shell, axis="x", min_gap=0.18)
    ctx.expect_overlap(backrest, seat_shell, axes="y", min_overlap=0.25)
    ctx.expect_overlap(leg_rest, seat_shell, axes="y", min_overlap=0.18)

    ctx.check(
        "back hinge is horizontal",
        abs(back_hinge.axis[0]) < 1e-9
        and abs(abs(back_hinge.axis[1]) - 1.0) < 1e-9
        and abs(back_hinge.axis[2]) < 1e-9,
        f"unexpected back hinge axis: {back_hinge.axis}",
    )
    ctx.check(
        "leg hinge is horizontal",
        abs(leg_hinge.axis[0]) < 1e-9
        and abs(abs(leg_hinge.axis[1]) - 1.0) < 1e-9
        and abs(leg_hinge.axis[2]) < 1e-9,
        f"unexpected leg hinge axis: {leg_hinge.axis}",
    )

    back_panel_rest = ctx.part_element_world_aabb(backrest, elem="back_panel")
    leg_panel_rest = ctx.part_element_world_aabb(leg_rest, elem="leg_panel")
    assert back_panel_rest is not None
    assert leg_panel_rest is not None

    with ctx.pose({back_hinge: 0.55}):
        back_panel_reclined = ctx.part_element_world_aabb(backrest, elem="back_panel")
        assert back_panel_reclined is not None
        ctx.check(
            "backrest reclines rearward",
            back_panel_reclined[0][0] < back_panel_rest[0][0] - 0.07,
            f"rearward shift too small: rest_min_x={back_panel_rest[0][0]:.3f}, reclined_min_x={back_panel_reclined[0][0]:.3f}",
        )

    with ctx.pose({leg_hinge: -0.65}):
        leg_panel_lowered = ctx.part_element_world_aabb(leg_rest, elem="leg_panel")
        assert leg_panel_lowered is not None
        ctx.check(
            "leg rest rotates independently downward",
            leg_panel_lowered[0][2] < leg_panel_rest[0][2] - 0.18,
            f"downward travel too small: rest_min_z={leg_panel_rest[0][2]:.3f}, lowered_min_z={leg_panel_lowered[0][2]:.3f}",
        )

    with ctx.pose({back_hinge: 0.55, leg_hinge: -0.65}):
        ctx.fail_if_parts_overlap_in_current_pose(name="articulated_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
