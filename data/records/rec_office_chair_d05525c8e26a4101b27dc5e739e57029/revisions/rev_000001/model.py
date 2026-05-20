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
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="meeting_room_chair")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.10, 0.11, 0.12, 1.0))
    shell_grey = model.material("shell_grey", rgba=(0.70, 0.72, 0.74, 1.0))
    fabric = model.material("fabric", rgba=(0.34, 0.37, 0.40, 1.0))
    polished = model.material("polished", rgba=(0.76, 0.78, 0.81, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _yz_section(
        width_y: float,
        height_z: float,
        radius: float,
        x_pos: float,
        *,
        z_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y_val, z_val + z_center)
            for y_val, z_val in rounded_rect_profile(
                width_y,
                height_z,
                radius,
                corner_segments=8,
            )
        ]

    def _xy_section(
        depth_x: float,
        width_y: float,
        radius: float,
        z_pos: float,
        *,
        x_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_val + x_center, y_val, z_pos)
            for x_val, y_val in rounded_rect_profile(
                depth_x,
                width_y,
                radius,
                corner_segments=8,
            )
        ]

    base = model.part("base")

    hub_profile = [
        (0.0, 0.000),
        (0.074, 0.012),
        (0.083, 0.024),
        (0.074, 0.052),
        (0.043, 0.088),
        (0.0, 0.104),
    ]
    base.visual(
        _mesh("meeting_room_chair_base_hub", LatheGeometry(hub_profile, segments=56)),
        material=graphite,
        name="base_hub_shell",
    )
    base.visual(
        Cylinder(radius=0.029, length=0.286),
        origin=Origin(xyz=(0.0, 0.0, 0.247)),
        material=graphite,
        name="pedestal_sleeve",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
        material=polished,
        name="gas_lift_inner",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.392)),
        material=graphite,
        name="pedestal_head",
    )

    arm_profile = rounded_rect_profile(0.070, 0.022, 0.008, corner_segments=6)
    arm_path = [
        (0.000, 0.000, 0.090),
        (0.115, 0.000, 0.094),
        (0.220, 0.000, 0.094),
        (0.302, 0.000, 0.091),
    ]
    base_arm_geom = sweep_profile_along_spline(
        arm_path,
        profile=arm_profile,
        samples_per_segment=10,
        cap_profile=True,
    )

    caster_layout = {
        "front": 0.0,
        "right": math.pi / 2.0,
        "rear": math.pi,
        "left": -math.pi / 2.0,
    }
    caster_radius = 0.302

    for label, angle in caster_layout.items():
        base.visual(
            _mesh(
                f"meeting_room_chair_{label}_base_arm",
                base_arm_geom.copy().rotate_z(angle),
            ),
            material=graphite,
            name=f"{label}_base_arm",
        )
        base.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(
                xyz=(
                    caster_radius * math.cos(angle),
                    caster_radius * math.sin(angle),
                    0.082,
                )
            ),
            material=dark_graphite,
            name=f"{label}_caster_socket_plate",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.690, 0.690, 0.410)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
    )

    seat_shell = model.part("seat_shell")
    seat_shell.visual(
        _mesh(
            "meeting_room_chair_seat_shell",
            section_loft(
                [
                    _yz_section(0.420, 0.040, 0.012, -0.175, z_center=0.036),
                    _yz_section(0.470, 0.050, 0.015, -0.070, z_center=0.034),
                    _yz_section(0.500, 0.048, 0.015, 0.060, z_center=0.028),
                    _yz_section(0.440, 0.040, 0.012, 0.230, z_center=0.034),
                ]
            ),
        ),
        material=shell_grey,
        name="seat_shell",
    )
    seat_shell.visual(
        Box((0.380, 0.405, 0.020)),
        origin=Origin(xyz=(0.020, 0.000, 0.038)),
        material=fabric,
        name="seat_pad",
    )
    seat_shell.visual(
        Box((0.160, 0.180, 0.026)),
        origin=Origin(xyz=(0.000, 0.000, 0.015)),
        material=dark_graphite,
        name="seat_support_box",
    )
    seat_shell.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=graphite,
        name="seat_mount_ring",
    )
    seat_shell.visual(
        Box((0.028, 0.285, 0.020)),
        origin=Origin(xyz=(0.214, 0.000, 0.022)),
        material=fabric,
        name="seat_front_lip",
    )
    seat_shell.visual(
        Box((0.028, 0.300, 0.022)),
        origin=Origin(xyz=(-0.173, 0.000, 0.028)),
        material=dark_graphite,
        name="seat_rear_bridge",
    )
    seat_shell.visual(
        Box((0.030, 0.050, 0.060)),
        origin=Origin(xyz=(-0.172, 0.154, 0.050)),
        material=dark_graphite,
        name="seat_right_hinge_cheek",
    )
    seat_shell.visual(
        Box((0.030, 0.050, 0.060)),
        origin=Origin(xyz=(-0.172, -0.154, 0.050)),
        material=dark_graphite,
        name="seat_left_hinge_cheek",
    )
    seat_shell.inertial = Inertial.from_geometry(
        Box((0.500, 0.500, 0.100)),
        mass=6.5,
        origin=Origin(xyz=(0.015, 0.000, 0.030)),
    )

    back_shell = model.part("back_shell")
    back_shell.visual(
        _mesh(
            "meeting_room_chair_back_shell",
            section_loft(
                [
                    _xy_section(0.040, 0.170, 0.010, 0.000, x_center=0.016),
                    _xy_section(0.050, 0.250, 0.012, 0.120, x_center=-0.004),
                    _xy_section(0.058, 0.440, 0.015, 0.270, x_center=-0.030),
                    _xy_section(0.054, 0.390, 0.014, 0.460, x_center=-0.072),
                ]
            ),
        ),
        material=shell_grey,
        name="back_shell",
    )
    back_shell.visual(
        Box((0.014, 0.315, 0.285)),
        origin=Origin(xyz=(-0.042, 0.000, 0.225)),
        material=fabric,
        name="back_pad",
    )
    back_shell.visual(
        Cylinder(radius=0.012, length=0.220),
        origin=Origin(xyz=(-0.018, 0.000, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="back_hinge_tube",
    )
    back_shell.visual(
        Box((0.024, 0.200, 0.040)),
        origin=Origin(xyz=(-0.006, 0.000, 0.050)),
        material=dark_graphite,
        name="back_lower_spine",
    )
    back_shell.inertial = Inertial.from_geometry(
        Box((0.160, 0.460, 0.520)),
        mass=5.0,
        origin=Origin(xyz=(-0.040, 0.000, 0.240)),
    )

    model.articulation(
        "base_to_shell_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat_shell,
        origin=Origin(xyz=(0.000, 0.000, 0.401)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=4.0),
    )
    model.articulation(
        "seat_to_back_recline",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=back_shell,
        origin=Origin(xyz=(-0.188, 0.000, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(18.0),
        ),
    )

    def _add_caster(label: str, angle: float) -> None:
        caster = model.part(f"{label}_caster")
        caster.visual(
            Cylinder(radius=0.0055, length=0.016),
            origin=Origin(xyz=(0.000, 0.000, -0.008)),
            material=polished,
            name="caster_stem",
        )
        caster.visual(
            Cylinder(radius=0.011, length=0.018),
            origin=Origin(xyz=(0.000, 0.000, -0.021)),
            material=graphite,
            name="caster_swivel_housing",
        )
        caster.visual(
            Box((0.018, 0.028, 0.006)),
            origin=Origin(xyz=(0.000, 0.000, -0.022)),
            material=graphite,
            name="caster_crown",
        )
        caster.visual(
            Box((0.012, 0.004, 0.030)),
            origin=Origin(xyz=(0.000, 0.012, -0.039)),
            material=graphite,
            name="caster_right_leg",
        )
        caster.visual(
            Box((0.012, 0.004, 0.030)),
            origin=Origin(xyz=(0.000, -0.012, -0.039)),
            material=graphite,
            name="caster_left_leg",
        )
        caster.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(xyz=(0.000, 0.010, -0.053), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name="caster_right_axle_cap",
        )
        caster.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(xyz=(0.000, -0.010, -0.053), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name="caster_left_axle_cap",
        )
        caster.inertial = Inertial.from_geometry(
            Box((0.030, 0.032, 0.090)),
            mass=0.30,
            origin=Origin(xyz=(0.000, 0.000, -0.050)),
        )

        wheel = model.part(f"{label}_caster_wheel")
        wheel.visual(
            Cylinder(radius=0.026, length=0.016),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="wheel_tire",
        )
        wheel.visual(
            Cylinder(radius=0.015, length=0.018),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_graphite,
            name="wheel_hub",
        )
        wheel.visual(
            Cylinder(radius=0.017, length=0.004),
            origin=Origin(xyz=(0.000, 0.008, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name="wheel_outer_cap",
        )
        wheel.visual(
            Cylinder(radius=0.017, length=0.004),
            origin=Origin(xyz=(0.000, -0.008, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name="wheel_inner_cap",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.026, length=0.016),
            mass=0.18,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"base_to_{label}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(
                xyz=(
                    caster_radius * math.cos(angle),
                    caster_radius * math.sin(angle),
                    0.079,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=10.0),
        )
        model.articulation(
            f"{label}_caster_to_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.000, 0.000, -0.053)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=35.0),
        )

    for label, angle in caster_layout.items():
        _add_caster(label, angle)

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

    base = object_model.get_part("base")
    seat_shell = object_model.get_part("seat_shell")
    back_shell = object_model.get_part("back_shell")

    shell_swivel = object_model.get_articulation("base_to_shell_swivel")
    back_recline = object_model.get_articulation("seat_to_back_recline")

    ctx.check(
        "shell assembly swivels on a vertical axis",
        shell_swivel.joint_type == ArticulationType.CONTINUOUS
        and tuple(shell_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={shell_swivel.joint_type}, axis={shell_swivel.axis}",
    )
    ctx.check(
        "back shell reclines on a transverse hinge",
        back_recline.joint_type == ArticulationType.REVOLUTE
        and tuple(back_recline.axis) == (0.0, -1.0, 0.0),
        details=f"type={back_recline.joint_type}, axis={back_recline.axis}",
    )

    for label in ("front", "right", "rear", "left"):
        caster_swivel = object_model.get_articulation(f"base_to_{label}_caster_swivel")
        wheel_spin = object_model.get_articulation(f"{label}_caster_to_wheel_spin")
        ctx.check(
            f"{label} caster stem swivels vertically",
            caster_swivel.joint_type == ArticulationType.CONTINUOUS
            and tuple(caster_swivel.axis) == (0.0, 0.0, 1.0),
            details=f"type={caster_swivel.joint_type}, axis={caster_swivel.axis}",
        )
        ctx.check(
            f"{label} caster wheel spins on a horizontal axle",
            wheel_spin.joint_type == ArticulationType.CONTINUOUS
            and tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
            details=f"type={wheel_spin.joint_type}, axis={wheel_spin.axis}",
        )

    ctx.expect_gap(
        seat_shell,
        base,
        axis="z",
        positive_elem="seat_mount_ring",
        negative_elem="pedestal_head",
        max_gap=0.001,
        max_penetration=0.0,
        name="seat shell is borne by the pedestal head",
    )
    ctx.expect_gap(
        seat_shell,
        back_shell,
        axis="x",
        positive_elem="seat_rear_bridge",
        negative_elem="back_hinge_tube",
        min_gap=0.002,
        max_gap=0.012,
        name="seat rear bridge sits just ahead of the back hinge tube",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][idx] + aabb[1][idx]) * 0.5 for idx in range(3))

    front_lip_rest = _aabb_center(ctx.part_element_world_aabb(seat_shell, elem="seat_front_lip"))
    with ctx.pose({shell_swivel: math.pi / 2.0}):
        front_lip_swiveled = _aabb_center(ctx.part_element_world_aabb(seat_shell, elem="seat_front_lip"))
    ctx.check(
        "seat shell rotates around the pedestal",
        front_lip_rest is not None
        and front_lip_swiveled is not None
        and front_lip_rest[0] > 0.16
        and abs(front_lip_rest[1]) < 0.03
        and front_lip_swiveled[1] > 0.16
        and abs(front_lip_swiveled[0]) < 0.03,
        details=f"rest={front_lip_rest}, swiveled={front_lip_swiveled}",
    )

    back_rest = _aabb_center(ctx.part_element_world_aabb(back_shell, elem="back_shell"))
    with ctx.pose({back_recline: math.radians(18.0)}):
        back_reclined = _aabb_center(ctx.part_element_world_aabb(back_shell, elem="back_shell"))
    ctx.check(
        "back shell reclines rearward",
        back_rest is not None
        and back_reclined is not None
        and back_reclined[0] < back_rest[0] - 0.04,
        details=f"rest={back_rest}, reclined={back_reclined}",
    )

    wheel_floor_ok = True
    wheel_floor_details: list[tuple[str, float | None]] = []
    for label in ("front", "right", "rear", "left"):
        wheel = object_model.get_part(f"{label}_caster_wheel")
        aabb = ctx.part_world_aabb(wheel)
        min_z = None if aabb is None else aabb[0][2]
        wheel_floor_details.append((wheel.name, min_z))
        if min_z is None or min_z < -0.002 or min_z > 0.015:
            wheel_floor_ok = False
    ctx.check(
        "caster wheels sit at floor height",
        wheel_floor_ok,
        details=str(wheel_floor_details),
    )

    caster_positions = []
    for label in ("front", "right", "rear", "left"):
        pos = ctx.part_world_position(object_model.get_part(f"{label}_caster"))
        caster_positions.append((label, pos))
    four_star_ok = (
        caster_positions[0][1] is not None
        and caster_positions[1][1] is not None
        and caster_positions[2][1] is not None
        and caster_positions[3][1] is not None
        and caster_positions[0][1][0] > 0.24
        and abs(caster_positions[0][1][1]) < 0.03
        and caster_positions[2][1][0] < -0.24
        and abs(caster_positions[2][1][1]) < 0.03
        and caster_positions[1][1][1] > 0.24
        and abs(caster_positions[1][1][0]) < 0.03
        and caster_positions[3][1][1] < -0.24
        and abs(caster_positions[3][1][0]) < 0.03
    )
    ctx.check(
        "four casters are arranged on a four-star base",
        four_star_ok,
        details=str(caster_positions),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
