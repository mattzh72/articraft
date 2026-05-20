from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = cos(angle)
    s = sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _add_wheel_visuals(part, *, wheel_prefix: str, rim_material, tire_material, hub_material) -> None:
    tire_radius = 0.20
    tire_width = 0.09
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))

    tire_profile = [
        (0.114, -0.043),
        (0.144, -0.045),
        (0.176, -0.040),
        (0.194, -0.024),
        (0.200, -0.010),
        (0.200, 0.010),
        (0.194, 0.024),
        (0.176, 0.040),
        (0.144, 0.045),
        (0.114, 0.043),
        (0.094, 0.018),
        (0.090, 0.000),
        (0.094, -0.018),
        (0.114, -0.043),
    ]
    tire_mesh = _save_mesh(
        f"{wheel_prefix}_tire.obj",
        LatheGeometry(tire_profile, segments=64).rotate_y(pi / 2.0),
    )
    part.visual(tire_mesh, material=tire_material, name="tire")

    rim_outer = superellipse_profile(0.245, 0.245, exponent=2.0, segments=44)
    spoke_window = rounded_rect_profile(0.040, 0.068, 0.010, corner_segments=5)
    hole_profiles = [superellipse_profile(0.072, 0.072, exponent=2.0, segments=20)]
    for spoke_index in range(5):
        angle = 2.0 * pi * spoke_index / 5.0
        hole_profiles.append(
            _transform_profile(
                spoke_window,
                dx=cos(angle) * 0.070,
                dy=sin(angle) * 0.070,
                angle=angle,
            )
        )

    rim_face = _save_mesh(
        f"{wheel_prefix}_rim_face.obj",
        ExtrudeWithHolesGeometry(
            rim_outer,
            hole_profiles,
            height=0.010,
            center=True,
        ).rotate_y(pi / 2.0),
    )
    part.visual(rim_face, origin=Origin(xyz=(0.020, 0.0, 0.0)), material=rim_material, name="rim_face_left")
    part.visual(rim_face, origin=Origin(xyz=(-0.020, 0.0, 0.0)), material=rim_material, name="rim_face_right")
    part.visual(Cylinder(radius=0.105, length=0.050), origin=spin_origin, material=hub_material, name="rim_barrel")
    part.visual(Cylinder(radius=0.038, length=0.072), origin=spin_origin, material=hub_material, name="hub_shell")
    part.visual(
        Cylinder(radius=0.030, length=0.009),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="left_spacer",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.009),
        origin=Origin(xyz=(-0.046, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="right_spacer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wheelbarrow", assets=ASSETS)

    tray_shell = model.material("tray_shell", rgba=(0.52, 0.55, 0.57, 1.0))
    tray_trim = model.material("tray_trim", rgba=(0.67, 0.69, 0.71, 1.0))
    frame_matte = model.material("frame_matte", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_hardware = model.material("satin_hardware", rgba=(0.63, 0.65, 0.68, 1.0))
    rim_satin = model.material("rim_satin", rgba=(0.76, 0.78, 0.80, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    rail_nodes = [
        (0.28, -0.72, 0.62),
        (0.28, -0.42, 0.56),
        (0.28, -0.20, 0.34),
        (0.28, 0.12, 0.28),
        (0.28, 0.42, 0.20),
        (0.24, 0.70, 0.20),
    ]
    leg_nodes = [
        (0.28, -0.20, 0.34),
        (0.30, 0.00, 0.24),
        (0.32, 0.06, 0.02),
    ]
    fork_nodes = [
        (0.24, 0.70, 0.20),
        (0.14, 0.84, 0.20),
        (0.093, 0.98, 0.20),
    ]

    frame_geom = tube_from_spline_points(
        rail_nodes,
        radius=0.018,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    frame_geom.merge(
        tube_from_spline_points(
            _mirror_x(rail_nodes),
            radius=0.018,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            leg_nodes,
            radius=0.016,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            _mirror_x(leg_nodes),
            radius=0.016,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            fork_nodes,
            radius=0.014,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            _mirror_x(fork_nodes),
            radius=0.014,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        )
    )
    frame_geom.merge(CylinderGeometry(radius=0.014, height=0.40).rotate_y(pi / 2.0).translate(0.0, -0.20, 0.34))
    frame_geom.merge(CylinderGeometry(radius=0.014, height=0.36).rotate_y(pi / 2.0).translate(0.0, 0.12, 0.28))
    frame_geom.merge(CylinderGeometry(radius=0.014, height=0.30).rotate_y(pi / 2.0).translate(0.0, 0.42, 0.20))
    frame_geom.merge(CylinderGeometry(radius=0.014, height=0.26).rotate_y(pi / 2.0).translate(0.0, 0.70, 0.20))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.54, 1.72, 0.68)),
        mass=15.5,
        origin=Origin(xyz=(0.0, 0.10, 0.30)),
    )
    frame.visual(_save_mesh("wheelbarrow_frame.obj", frame_geom), material=frame_matte, name="frame_loop")
    frame.visual(
        Box((0.12, 0.26, 0.17)),
        origin=Origin(xyz=(0.22, 0.18, 0.345)),
        material=frame_matte,
        name="left_support_tower",
    )
    frame.visual(
        Box((0.12, 0.26, 0.17)),
        origin=Origin(xyz=(-0.22, 0.18, 0.345)),
        material=frame_matte,
        name="right_support_tower",
    )
    frame.visual(
        Box((0.012, 0.036, 0.064)),
        origin=Origin(xyz=(0.076, 0.98, 0.20)),
        material=satin_hardware,
        name="left_axle_plate",
    )
    frame.visual(
        Box((0.012, 0.036, 0.064)),
        origin=Origin(xyz=(-0.076, 0.98, 0.20)),
        material=satin_hardware,
        name="right_axle_plate",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.088, 0.98, 0.20), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_hardware,
        name="left_axle_nut",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(-0.088, 0.98, 0.20), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_hardware,
        name="right_axle_nut",
    )
    frame.visual(
        Cylinder(radius=0.021, length=0.16),
        origin=Origin(xyz=(0.28, -0.74, 0.62), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.021, length=0.16),
        origin=Origin(xyz=(-0.28, -0.74, 0.62), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.56, 0.84, 0.26)),
        mass=8.3,
        origin=Origin(xyz=(0.0, 0.05, 0.52)),
    )
    tray.visual(
        Box((0.50, 0.74, 0.008)),
        origin=Origin(xyz=(0.0, 0.05, 0.462)),
        material=tray_shell,
        name="floor",
    )
    tray.visual(
        Box((0.008, 0.78, 0.16)),
        origin=Origin(xyz=(0.246, 0.05, 0.546)),
        material=tray_shell,
        name="left_side_wall",
    )
    tray.visual(
        Box((0.008, 0.78, 0.16)),
        origin=Origin(xyz=(-0.246, 0.05, 0.546)),
        material=tray_shell,
        name="right_side_wall",
    )
    tray.visual(
        Box((0.36, 0.008, 0.16)),
        origin=Origin(xyz=(0.0, 0.424, 0.546)),
        material=tray_shell,
        name="front_wall",
    )
    tray.visual(
        Box((0.44, 0.008, 0.08)),
        origin=Origin(xyz=(0.0, -0.324, 0.506)),
        material=tray_shell,
        name="rear_wall",
    )
    tray.visual(
        Box((0.024, 0.78, 0.012)),
        origin=Origin(xyz=(0.246, 0.05, 0.632)),
        material=tray_shell,
        name="left_lip",
    )
    tray.visual(
        Box((0.024, 0.78, 0.012)),
        origin=Origin(xyz=(-0.246, 0.05, 0.632)),
        material=tray_shell,
        name="right_lip",
    )
    tray.visual(
        Box((0.384, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.424, 0.632)),
        material=tray_shell,
        name="front_lip",
    )
    tray.visual(
        Box((0.464, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, -0.324, 0.552)),
        material=tray_shell,
        name="rear_lip",
    )
    tray.visual(
        _save_mesh(
            "wheelbarrow_tray_rolled_lip.obj",
            wire_from_points(
                [
                    (-0.232, -0.324, 0.552),
                    (-0.258, -0.12, 0.610),
                    (-0.258, 0.18, 0.632),
                    (-0.195, 0.424, 0.632),
                    (0.0, 0.448, 0.632),
                    (0.195, 0.424, 0.632),
                    (0.258, 0.18, 0.632),
                    (0.258, -0.12, 0.610),
                    (0.232, -0.324, 0.552),
                ],
                radius=0.009,
                radial_segments=18,
                closed_path=True,
                cap_ends=False,
                corner_mode="fillet",
                corner_radius=0.035,
                corner_segments=8,
            ),
        ),
        material=tray_trim,
        name="rolled_lip",
    )
    tray.visual(
        Box((0.05, 0.58, 0.028)),
        origin=Origin(xyz=(0.19, 0.05, 0.444)),
        material=tray_trim,
        name="left_under_rib",
    )
    tray.visual(
        Box((0.05, 0.58, 0.028)),
        origin=Origin(xyz=(-0.19, 0.05, 0.444)),
        material=tray_trim,
        name="right_under_rib",
    )
    tray.visual(
        Box((0.22, 0.16, 0.028)),
        origin=Origin(xyz=(0.0, 0.30, 0.444)),
        material=tray_trim,
        name="nose_rib",
    )
    tray.visual(
        Box((0.06, 0.12, 0.062)),
        origin=Origin(xyz=(0.18, 0.18, 0.461)),
        material=tray_trim,
        name="left_mount_block",
    )
    tray.visual(
        Box((0.06, 0.12, 0.062)),
        origin=Origin(xyz=(-0.18, 0.18, 0.461)),
        material=tray_trim,
        name="right_mount_block",
    )
    tray.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=Origin(xyz=(0.225, 0.18, 0.461), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_hardware,
        name="left_pivot_sleeve",
    )
    tray.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=Origin(xyz=(-0.225, 0.18, 0.461), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_hardware,
        name="right_pivot_sleeve",
    )

    wheel = model.part("front_wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.08),
        mass=3.4,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    tire_profile = [
        (0.115, -0.038),
        (0.150, -0.040),
        (0.180, -0.035),
        (0.196, -0.020),
        (0.200, -0.008),
        (0.200, 0.008),
        (0.196, 0.020),
        (0.180, 0.035),
        (0.150, 0.040),
        (0.115, 0.038),
        (0.108, 0.015),
        (0.108, 0.000),
        (0.108, -0.015),
        (0.115, -0.038),
    ]
    wheel_core = CylinderGeometry(radius=0.110, height=0.120).rotate_y(pi / 2.0)
    wheel_core.merge(CylinderGeometry(radius=0.152, height=0.012).rotate_y(pi / 2.0).translate(0.022, 0.0, 0.0))
    wheel_core.merge(CylinderGeometry(radius=0.152, height=0.012).rotate_y(pi / 2.0).translate(-0.022, 0.0, 0.0))
    wheel_core.merge(CylinderGeometry(radius=0.036, height=0.140).rotate_y(pi / 2.0))
    wheel.visual(
        _save_mesh("wheelbarrow_front_tire.obj", LatheGeometry(tire_profile, segments=64).rotate_y(pi / 2.0)),
        material=tire_rubber,
        name="tire",
    )
    wheel.visual(
        _save_mesh("wheelbarrow_front_wheel_core.obj", wheel_core),
        material=rim_satin,
        name="hub_shell",
    )
    rim_outer = superellipse_profile(0.292, 0.292, exponent=2.0, segments=40)
    spoke_window = rounded_rect_profile(0.030, 0.055, 0.008, corner_segments=5)
    hole_profiles = [superellipse_profile(0.070, 0.070, exponent=2.0, segments=20)]
    for spoke_index in range(5):
        angle = 2.0 * pi * spoke_index / 5.0
        hole_profiles.append(
            _transform_profile(
                spoke_window,
                dx=cos(angle) * 0.072,
                dy=sin(angle) * 0.072,
                angle=angle,
            )
        )
    rim_face = _save_mesh(
        "wheelbarrow_front_rim_face.obj",
        ExtrudeWithHolesGeometry(rim_outer, hole_profiles, height=0.008, center=True).rotate_y(pi / 2.0),
    )
    wheel.visual(rim_face, origin=Origin(xyz=(0.024, 0.0, 0.0)), material=rim_satin, name="rim_face_left")
    wheel.visual(rim_face, origin=Origin(xyz=(-0.024, 0.0, 0.0)), material=rim_satin, name="rim_face_right")
    wheel.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_hardware,
        name="axle_cap",
    )

    model.articulation(
        "frame_to_tray",
        ArticulationType.FIXED,
        parent=frame,
        child=tray,
        origin=Origin(),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.98, 0.20)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    wheel = object_model.get_part("front_wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")

    left_under_rib = tray.get_visual("left_under_rib")
    right_under_rib = tray.get_visual("right_under_rib")
    left_mount_block = tray.get_visual("left_mount_block")
    right_mount_block = tray.get_visual("right_mount_block")
    front_wall = tray.get_visual("front_wall")
    left_support_tower = frame.get_visual("left_support_tower")
    right_support_tower = frame.get_visual("right_support_tower")
    left_axle_plate = frame.get_visual("left_axle_plate")
    right_axle_plate = frame.get_visual("right_axle_plate")
    left_grip = frame.get_visual("left_grip")
    hub_shell = wheel.get_visual("hub_shell")
    tire = wheel.get_visual("tire")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=10, name="articulation_pose_clearance")

    ctx.expect_contact(tray, frame, elem_a=left_mount_block, elem_b=left_support_tower, name="left_tray_support_contact")
    ctx.expect_contact(
        tray,
        frame,
        elem_a=right_mount_block,
        elem_b=right_support_tower,
        name="right_tray_support_contact",
    )
    ctx.expect_contact(wheel, frame, elem_a=hub_shell, elem_b=left_axle_plate, name="left_wheel_support_contact")
    ctx.expect_contact(wheel, frame, elem_a=hub_shell, elem_b=right_axle_plate, name="right_wheel_support_contact")
    ctx.expect_overlap(tray, frame, axes="xy", min_overlap=0.30, name="tray_over_frame_footprint")
    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem=left_grip,
        negative_elem=tire,
        min_gap=0.16,
        max_gap=0.30,
        name="left_grip_above_wheel",
    )
    ctx.expect_gap(
        wheel,
        tray,
        axis="y",
        positive_elem=tire,
        negative_elem=front_wall,
        min_gap=0.22,
        max_gap=0.50,
        name="wheel_forward_of_tray_nose",
    )

    with ctx.pose({wheel_spin: 1.6}):
        ctx.expect_contact(
            wheel,
            frame,
            elem_a=hub_shell,
            elem_b=left_axle_plate,
            name="left_wheel_support_contact_spun",
        )
        ctx.expect_contact(
            wheel,
            frame,
            elem_a=hub_shell,
            elem_b=right_axle_plate,
            name="right_wheel_support_contact_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
