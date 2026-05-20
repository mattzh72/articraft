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
    section_loft,
)


def _body_section(
    x_pos: float,
    *,
    width: float,
    belt_z: float,
    roof_z: float,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    return [
        (x_pos, 0.22 * half_w, 0.0),
        (x_pos, 0.52 * half_w, 0.004),
        (x_pos, 1.00 * half_w, belt_z),
        (x_pos, 0.58 * half_w, roof_z * 0.86),
        (x_pos, 0.16 * half_w, roof_z),
        (x_pos, -0.16 * half_w, roof_z),
        (x_pos, -0.58 * half_w, roof_z * 0.86),
        (x_pos, -1.00 * half_w, belt_z),
        (x_pos, -0.52 * half_w, 0.004),
        (x_pos, -0.22 * half_w, 0.0),
    ]


def _hood_section(
    x_pos: float,
    *,
    width: float,
    top_z: float,
    lower_z: float,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    return [
        (x_pos, 0.12 * half_w, top_z),
        (x_pos, 0.55 * half_w, top_z - 0.001),
        (x_pos, 0.96 * half_w, lower_z + 0.001),
        (x_pos, 0.60 * half_w, lower_z),
        (x_pos, 0.00, lower_z),
        (x_pos, -0.60 * half_w, lower_z),
        (x_pos, -0.96 * half_w, lower_z + 0.001),
        (x_pos, -0.55 * half_w, top_z - 0.001),
        (x_pos, -0.12 * half_w, top_z),
        (x_pos, 0.00, top_z + 0.0005),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_car")

    chassis_black = model.material("chassis_black", rgba=(0.12, 0.12, 0.13, 1.0))
    body_red = model.material("body_red", rgba=(0.78, 0.12, 0.12, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    hub_grey = model.material("hub_grey", rgba=(0.63, 0.65, 0.68, 1.0))
    mount_grey = model.material("mount_grey", rgba=(0.34, 0.35, 0.37, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.170, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=chassis_black,
        name="undertray",
    )
    chassis.visual(
        Box((0.078, 0.022, 0.008)),
        origin=Origin(xyz=(-0.010, 0.0, 0.019)),
        material=chassis_black,
        name="center_spine",
    )
    chassis.visual(
        Box((0.032, 0.046, 0.010)),
        origin=Origin(xyz=(0.066, 0.0, 0.024)),
        material=chassis_black,
        name="front_crossmember",
    )
    chassis.visual(
        Box((0.042, 0.042, 0.012)),
        origin=Origin(xyz=(-0.060, 0.0, 0.018)),
        material=chassis_black,
        name="rear_block",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((0.170, 0.050, 0.030)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    body_shell = model.part("body_shell")
    body_geom = section_loft(
        [
            _body_section(-0.074, width=0.052, belt_z=0.010, roof_z=0.018),
            _body_section(-0.048, width=0.070, belt_z=0.018, roof_z=0.028),
            _body_section(-0.010, width=0.078, belt_z=0.021, roof_z=0.034),
            _body_section(0.020, width=0.066, belt_z=0.018, roof_z=0.026),
        ]
    )
    body_shell.visual(
        mesh_from_geometry(body_geom, "toy_car_body_shell"),
        material=body_red,
        name="cabin_shell",
    )
    body_shell.visual(
        Box((0.120, 0.010, 0.014)),
        origin=Origin(xyz=(-0.010, 0.022, 0.007)),
        material=body_red,
        name="left_rocker",
    )
    body_shell.visual(
        Box((0.120, 0.010, 0.014)),
        origin=Origin(xyz=(-0.010, -0.022, 0.007)),
        material=body_red,
        name="right_rocker",
    )
    body_shell.visual(
        Box((0.016, 0.048, 0.006)),
        origin=Origin(xyz=(0.007, 0.0, 0.018)),
        material=body_red,
        name="cowl_bridge",
    )
    body_shell.visual(
        Box((0.074, 0.008, 0.010)),
        origin=Origin(xyz=(0.052, 0.019, 0.012)),
        material=body_red,
        name="left_hood_rail",
    )
    body_shell.visual(
        Box((0.074, 0.008, 0.010)),
        origin=Origin(xyz=(0.052, -0.019, 0.012)),
        material=body_red,
        name="right_hood_rail",
    )
    body_shell.visual(
        Box((0.018, 0.040, 0.012)),
        origin=Origin(xyz=(0.086, 0.0, 0.010)),
        material=body_red,
        name="nose_block",
    )
    body_shell.visual(
        Box((0.022, 0.054, 0.012)),
        origin=Origin(xyz=(-0.083, 0.0, 0.008)),
        material=body_red,
        name="rear_tail",
    )
    body_shell.inertial = Inertial.from_geometry(
        Box((0.190, 0.080, 0.040)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    hood = model.part("hood")
    hood_geom = section_loft(
        [
            _hood_section(0.000, width=0.034, top_z=0.0, lower_z=-0.0035),
            _hood_section(0.028, width=0.036, top_z=-0.002, lower_z=-0.006),
            _hood_section(0.056, width=0.031, top_z=-0.0065, lower_z=-0.010),
        ]
    )
    hood.visual(
        mesh_from_geometry(hood_geom, "toy_car_hood_panel"),
        material=body_red,
        name="hood_panel",
    )
    hood.visual(
        Box((0.010, 0.034, 0.004)),
        origin=Origin(xyz=(-0.001, 0.0, -0.002)),
        material=body_red,
        name="hinge_lip",
    )
    hood.inertial = Inertial.from_geometry(
        Box((0.058, 0.036, 0.010)),
        mass=0.08,
        origin=Origin(xyz=(0.028, 0.0, -0.005)),
    )

    def add_mount_and_wheel(
        mount_name: str,
        wheel_name: str,
        *,
        x_pos: float,
        side_sign: float,
    ) -> None:
        mount = model.part(mount_name)
        mount.visual(
            Box((0.010, 0.014, 0.018)),
            origin=Origin(xyz=(0.0, side_sign * 0.007, 0.0)),
            material=mount_grey,
            name="upright",
        )
        mount.visual(
            Cylinder(radius=0.0035, length=0.011),
            origin=Origin(
                xyz=(0.0, side_sign * 0.0195, -0.001),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=mount_grey,
            name="axle_stub",
        )
        mount.inertial = Inertial.from_geometry(
            Box((0.012, 0.026, 0.020)),
            mass=0.03,
            origin=Origin(xyz=(0.0, side_sign * 0.010, 0.0)),
        )

        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.021, length=0.014),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.015),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hub_grey,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.005, length=0.017),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mount_grey,
            name="hub_cap",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.021, length=0.014),
            mass=0.08,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"chassis_to_{mount_name}",
            ArticulationType.FIXED,
            parent=chassis,
            child=mount,
            origin=Origin(xyz=(x_pos, side_sign * 0.025, 0.020)),
        )
        model.articulation(
            f"{mount_name}_to_{wheel_name}",
            ArticulationType.FIXED,
            parent=mount,
            child=wheel,
            origin=Origin(xyz=(0.0, side_sign * 0.032, -0.001)),
        )

    add_mount_and_wheel(
        "front_left_mount",
        "front_left_wheel",
        x_pos=0.055,
        side_sign=1.0,
    )
    add_mount_and_wheel(
        "front_right_mount",
        "front_right_wheel",
        x_pos=0.055,
        side_sign=-1.0,
    )
    add_mount_and_wheel(
        "rear_left_mount",
        "rear_left_wheel",
        x_pos=-0.055,
        side_sign=1.0,
    )
    add_mount_and_wheel(
        "rear_right_mount",
        "rear_right_wheel",
        x_pos=-0.055,
        side_sign=-1.0,
    )

    model.articulation(
        "chassis_to_body_shell",
        ArticulationType.FIXED,
        parent=chassis,
        child=body_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )
    model.articulation(
        "body_shell_to_hood",
        ArticulationType.REVOLUTE,
        parent=body_shell,
        child=hood,
        origin=Origin(xyz=(0.021, 0.0, 0.021)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    body_shell = object_model.get_part("body_shell")
    hood = object_model.get_part("hood")
    hood_hinge = object_model.get_articulation("body_shell_to_hood")
    front_left_mount = object_model.get_part("front_left_mount")
    front_right_mount = object_model.get_part("front_right_mount")
    rear_left_mount = object_model.get_part("rear_left_mount")
    rear_right_mount = object_model.get_part("rear_right_mount")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

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

    ctx.expect_contact(body_shell, chassis, name="body shell is mounted on the chassis")

    for mount in (
        front_left_mount,
        front_right_mount,
        rear_left_mount,
        rear_right_mount,
    ):
        ctx.expect_contact(mount, chassis, name=f"{mount.name} contacts the chassis")

    for wheel, mount in (
        (front_left_wheel, front_left_mount),
        (front_right_wheel, front_right_mount),
        (rear_left_wheel, rear_left_mount),
        (rear_right_wheel, rear_right_mount),
    ):
        ctx.expect_contact(wheel, mount, name=f"{wheel.name} sits on its wheel mount")

    ctx.check(
        "hood hinge axis opens upward",
        tuple(round(v, 6) for v in hood_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hood_hinge.axis}",
    )

    with ctx.pose({hood_hinge: 0.0}):
        ctx.expect_contact(
            hood,
            body_shell,
            elem_a="hinge_lip",
            elem_b="cowl_bridge",
            name="closed hood touches the cowl bridge",
        )
        ctx.expect_overlap(
            hood,
            body_shell,
            axes="xy",
            min_overlap=0.020,
            name="hood stays nested within the front body footprint",
        )
        hood_closed_aabb = ctx.part_element_world_aabb(hood, elem="hood_panel")

    with ctx.pose({hood_hinge: 0.95}):
        hood_open_aabb = ctx.part_element_world_aabb(hood, elem="hood_panel")

    ctx.check(
        "hood rises when opened",
        hood_closed_aabb is not None
        and hood_open_aabb is not None
        and hood_open_aabb[1][2] > hood_closed_aabb[1][2] + 0.020,
        details=f"closed={hood_closed_aabb}, open={hood_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
