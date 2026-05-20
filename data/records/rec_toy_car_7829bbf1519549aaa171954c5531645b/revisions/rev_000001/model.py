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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xz_section(
    width: float,
    height: float,
    radius: float,
    y_pos: float,
    *,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y_pos, z + z_center) for x, z in rounded_rect_profile(width, height, radius)]


def _yz_section(
    length: float,
    height: float,
    radius: float,
    x_pos: float,
    *,
    y_center: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y + y_center, z + z_center) for y, z in rounded_rect_profile(length, height, radius)]


def _add_wheel_visuals(
    part,
    mesh_name: str,
    *,
    side_sign: float,
    tire_radius: float,
    tire_width: float,
    rim_radius: float,
    rubber,
    wheel_silver,
    dark_trim,
) -> None:
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.58, -half_width * 0.98),
        (tire_radius * 0.78, -half_width),
        (tire_radius * 0.95, -half_width * 0.78),
        (tire_radius, -half_width * 0.30),
        (tire_radius, half_width * 0.30),
        (tire_radius * 0.95, half_width * 0.78),
        (tire_radius * 0.78, half_width),
        (tire_radius * 0.58, half_width * 0.98),
        (tire_radius * 0.44, half_width * 0.40),
        (tire_radius * 0.42, 0.0),
        (tire_radius * 0.44, -half_width * 0.40),
        (tire_radius * 0.58, -half_width * 0.98),
    ]
    part.visual(
        _save_mesh(mesh_name, LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0)),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=tire_width * 0.62),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=rim_radius * 0.92, length=0.004),
        origin=Origin(
            xyz=(-side_sign * (half_width - 0.002), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=wheel_silver,
        name="hub_inner",
    )
    part.visual(
        Cylinder(radius=rim_radius * 0.92, length=0.004),
        origin=Origin(
            xyz=(side_sign * (half_width - 0.002), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=wheel_silver,
        name="hub_outer",
    )
    part.visual(
        Cylinder(radius=rim_radius * 0.38, length=tire_width * 0.80),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_silver,
        name="hub_core",
    )
    part.visual(
        Cylinder(radius=rim_radius * 0.14, length=tire_width * 0.90),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="axle_bore",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_race_car")

    body_red = model.material("body_red", rgba=(0.84, 0.10, 0.08, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.20, 0.21, 0.22, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.76, 0.78, 0.82, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.42, 0.58, 0.76, 0.45))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((0.24, 0.32, 0.08)),
        mass=0.90,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    body_sections = [
        _xz_section(0.020, 0.006, 0.0018, 0.148, z_center=0.010),
        _xz_section(0.068, 0.010, 0.0030, 0.112, z_center=0.011),
        _xz_section(0.112, 0.016, 0.0045, 0.055, z_center=0.014),
        _xz_section(0.124, 0.018, 0.0055, -0.010, z_center=0.015),
        _xz_section(0.122, 0.019, 0.0055, -0.070, z_center=0.016),
        _xz_section(0.110, 0.018, 0.0050, -0.118, z_center=0.017),
        _xz_section(0.080, 0.012, 0.0035, -0.150, z_center=0.016),
    ]
    chassis.visual(
        _save_mesh("race_car_body_shell", section_loft(body_sections)),
        material=body_red,
        name="body_shell",
    )
    chassis.visual(
        Box((0.108, 0.210, 0.008)),
        origin=Origin(xyz=(0.0, -0.010, 0.006)),
        material=trim_black,
        name="floor_pan",
    )
    chassis.visual(
        Box((0.056, 0.034, 0.003)),
        origin=Origin(xyz=(0.0, 0.142, 0.006)),
        material=trim_black,
        name="front_splitter",
    )
    chassis.visual(
        Box((0.050, 0.074, 0.012)),
        origin=Origin(xyz=(0.0, -0.006, 0.020)),
        material=dark_trim,
        name="cockpit_tub",
    )
    chassis.visual(
        Box((0.008, 0.090, 0.011)),
        origin=Origin(xyz=(0.033, -0.006, 0.0255)),
        material=body_red,
        name="left_canopy_sill",
    )
    chassis.visual(
        Box((0.008, 0.090, 0.011)),
        origin=Origin(xyz=(-0.033, -0.006, 0.0255)),
        material=body_red,
        name="right_canopy_sill",
    )
    chassis.visual(
        Box((0.014, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.060, 0.023)),
        material=dark_trim,
        name="roll_hoop",
    )
    chassis.visual(
        Box((0.022, 0.190, 0.014)),
        origin=Origin(xyz=(0.051, -0.005, 0.020)),
        material=body_red,
        name="left_side_pod",
    )
    chassis.visual(
        Box((0.022, 0.190, 0.014)),
        origin=Origin(xyz=(-0.051, -0.005, 0.020)),
        material=body_red,
        name="right_side_pod",
    )
    chassis.visual(
        Box((0.082, 0.062, 0.006)),
        origin=Origin(xyz=(0.0, -0.082, 0.023)),
        material=body_red,
        name="engine_deck",
    )
    chassis.visual(
        Box((0.052, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, -0.080, 0.019)),
        material=dark_trim,
        name="engine_block",
    )
    chassis.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.016, -0.088, 0.026), rpy=(0.0, 0.0, 0.0)),
        material=wheel_silver,
        name="left_intake",
    )
    chassis.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(-0.016, -0.088, 0.026), rpy=(0.0, 0.0, 0.0)),
        material=wheel_silver,
        name="right_intake",
    )
    chassis.visual(
        Box((0.010, 0.020, 0.016)),
        origin=Origin(xyz=(0.028, -0.126, 0.024)),
        material=dark_trim,
        name="rear_hinge_left_pedestal",
    )
    chassis.visual(
        Box((0.010, 0.020, 0.016)),
        origin=Origin(xyz=(-0.028, -0.126, 0.024)),
        material=dark_trim,
        name="rear_hinge_right_pedestal",
    )

    wheel_mounts = [
        ("front_left", 0.078, 0.095),
        ("front_right", -0.078, 0.095),
        ("rear_left", 0.078, -0.095),
        ("rear_right", -0.078, -0.095),
    ]
    for prefix, x_pos, y_pos in wheel_mounts:
        side_sign = 1.0 if x_pos > 0.0 else -1.0
        chassis.visual(
            Box((0.022, 0.022, 0.020)),
            origin=Origin(xyz=(0.050 * side_sign, y_pos, 0.037)),
            material=dark_trim,
            name=f"{prefix}_support_arm",
        )
        chassis.visual(
            Cylinder(radius=0.008, length=0.018),
            origin=Origin(xyz=(0.052 * side_sign, y_pos, 0.037), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_silver,
            name=f"{prefix}_axle_stub",
        )

    canopy = model.part("canopy")
    canopy.visual(
        _save_mesh(
            "race_car_canopy_shell",
            section_loft(
                [
                    _yz_section(0.076, 0.011, 0.003, 0.000, y_center=0.012, z_center=0.006),
                    _yz_section(0.096, 0.024, 0.007, -0.018, y_center=0.012, z_center=0.013),
                    _yz_section(0.098, 0.031, 0.009, -0.040, y_center=0.012, z_center=0.018),
                    _yz_section(0.072, 0.017, 0.005, -0.062, y_center=0.012, z_center=0.011),
                ]
            ),
        ),
        material=canopy_tint,
        name="canopy_shell",
    )
    canopy.visual(
        Box((0.006, 0.076, 0.008)),
        origin=Origin(xyz=(-0.003, 0.012, 0.006)),
        material=trim_black,
        name="canopy_base_rail",
    )
    canopy.visual(
        Cylinder(radius=0.004, length=0.046),
        origin=Origin(xyz=(-0.001, 0.012, 0.006), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((0.072, 0.120, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(-0.036, -0.010, 0.018)),
    )

    engine_cover = model.part("engine_cover")
    engine_cover.visual(
        _save_mesh(
            "race_car_engine_cover",
            section_loft(
                [
                    _xz_section(0.056, 0.009, 0.003, 0.000, z_center=0.005),
                    _xz_section(0.070, 0.015, 0.005, 0.026, z_center=0.010),
                    _xz_section(0.066, 0.017, 0.006, 0.054, z_center=0.012),
                    _xz_section(0.050, 0.009, 0.003, 0.076, z_center=0.007),
                ]
            ),
        ),
        material=body_red,
        name="cover_shell",
    )
    engine_cover.visual(
        Cylinder(radius=0.004, length=0.060),
        origin=Origin(xyz=(0.0, 0.002, 0.006), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="cover_hinge_barrel",
    )
    engine_cover.visual(
        Box((0.046, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.016, 0.010)),
        material=dark_trim,
        name="cover_inner_lip",
    )
    engine_cover.inertial = Inertial.from_geometry(
        Box((0.090, 0.110, 0.030)),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.052, 0.013)),
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.034),
        mass=0.05,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        front_left_wheel,
        "front_left_wheel_tire",
        side_sign=1.0,
        tire_radius=0.037,
        tire_width=0.034,
        rim_radius=0.026,
        rubber=rubber,
        wheel_silver=wheel_silver,
        dark_trim=dark_trim,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.034),
        mass=0.05,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        front_right_wheel,
        "front_right_wheel_tire",
        side_sign=-1.0,
        tire_radius=0.037,
        tire_width=0.034,
        rim_radius=0.026,
        rubber=rubber,
        wheel_silver=wheel_silver,
        dark_trim=dark_trim,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.034),
        mass=0.05,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_left_wheel,
        "rear_left_wheel_tire",
        side_sign=1.0,
        tire_radius=0.037,
        tire_width=0.034,
        rim_radius=0.026,
        rubber=rubber,
        wheel_silver=wheel_silver,
        dark_trim=dark_trim,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.034),
        mass=0.05,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_right_wheel,
        "rear_right_wheel_tire",
        side_sign=-1.0,
        tire_radius=0.037,
        tire_width=0.034,
        rim_radius=0.026,
        rubber=rubber,
        wheel_silver=wheel_silver,
        dark_trim=dark_trim,
    )

    model.articulation(
        "chassis_to_canopy",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=canopy,
        origin=Origin(xyz=(0.033, -0.004, 0.029)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "chassis_to_engine_cover",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=engine_cover,
        origin=Origin(xyz=(0.0, -0.126, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    wheel_positions = {
        "front_left_wheel_spin": (front_left_wheel, 0.078, 0.095),
        "front_right_wheel_spin": (front_right_wheel, -0.078, 0.095),
        "rear_left_wheel_spin": (rear_left_wheel, 0.078, -0.095),
        "rear_right_wheel_spin": (rear_right_wheel, -0.078, -0.095),
    }
    for joint_name, (wheel_part, x_pos, y_pos) in wheel_positions.items():
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=wheel_part,
            origin=Origin(xyz=(x_pos, y_pos, 0.037)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=18.0),
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

    chassis = object_model.get_part("chassis")
    canopy = object_model.get_part("canopy")
    engine_cover = object_model.get_part("engine_cover")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    canopy_hinge = object_model.get_articulation("chassis_to_canopy")
    engine_cover_hinge = object_model.get_articulation("chassis_to_engine_cover")
    front_left_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_spin = object_model.get_articulation("front_right_wheel_spin")
    rear_left_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_spin = object_model.get_articulation("rear_right_wheel_spin")

    ctx.check(
        "all wheel joints are continuous axle spins",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (1.0, 0.0, 0.0)
            for joint in (front_left_spin, front_right_spin, rear_left_spin, rear_right_spin)
        ),
        details=str(
            [
                (joint.name, joint.articulation_type, joint.axis)
                for joint in (front_left_spin, front_right_spin, rear_left_spin, rear_right_spin)
            ]
        ),
    )
    ctx.check(
        "canopy and engine cover hinges use realistic axes",
        canopy_hinge.axis == (0.0, 1.0, 0.0) and engine_cover_hinge.axis == (1.0, 0.0, 0.0),
        details=f"canopy_axis={canopy_hinge.axis}, cover_axis={engine_cover_hinge.axis}",
    )

    wheel_mounts = [
        (front_left_wheel, "front_left_axle_stub"),
        (front_right_wheel, "front_right_axle_stub"),
        (rear_left_wheel, "rear_left_axle_stub"),
        (rear_right_wheel, "rear_right_axle_stub"),
    ]
    with ctx.pose(
        {
            canopy_hinge: 0.0,
            engine_cover_hinge: 0.0,
            front_left_spin: 0.6,
            front_right_spin: -0.7,
            rear_left_spin: 1.2,
            rear_right_spin: -1.3,
        }
    ):
        for wheel, axle_stub in wheel_mounts:
            ctx.expect_contact(
                wheel,
                chassis,
                elem_a="hub_inner",
                elem_b=axle_stub,
                contact_tol=0.001,
                name=f"{wheel.name} stays mounted on {axle_stub}",
            )
        ctx.expect_overlap(
            canopy,
            chassis,
            axes="xy",
            min_overlap=0.035,
            elem_a="canopy_shell",
            elem_b="cockpit_tub",
            name="closed canopy covers the cockpit tub",
        )
        ctx.expect_overlap(
            engine_cover,
            chassis,
            axes="xy",
            min_overlap=0.035,
            elem_a="cover_shell",
            elem_b="engine_deck",
            name="closed engine cover sits over the rear deck",
        )

    closed_canopy_aabb = ctx.part_element_world_aabb(canopy, elem="canopy_shell")
    with ctx.pose({canopy_hinge: 1.0}):
        open_canopy_aabb = ctx.part_element_world_aabb(canopy, elem="canopy_shell")
    ctx.check(
        "canopy opens upward on its side hinge",
        closed_canopy_aabb is not None
        and open_canopy_aabb is not None
        and open_canopy_aabb[1][2] > closed_canopy_aabb[1][2] + 0.025,
        details=f"closed={closed_canopy_aabb}, open={open_canopy_aabb}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(engine_cover, elem="cover_shell")
    with ctx.pose({engine_cover_hinge: 0.9}):
        open_cover_aabb = ctx.part_element_world_aabb(engine_cover, elem="cover_shell")
    ctx.check(
        "rear engine cover lifts upward from the rear hinge",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.020,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
