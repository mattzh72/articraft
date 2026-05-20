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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="postal_platform_cart")

    platform_length = 0.92
    platform_width = 0.58
    frame_top_z = 0.158
    frame_bottom_z = 0.128
    frame_center_z = (frame_top_z + frame_bottom_z) * 0.5
    deck_top_z = 0.173
    deck_thickness = 0.016
    deck_center_z = deck_top_z - deck_thickness * 0.5
    hinge_z = 0.188

    caster_radius = 0.062
    caster_width = 0.034
    caster_tire_width = 0.026
    caster_hub_radius = 0.036
    caster_swivel_drop = 0.066
    caster_trail = 0.072

    cart_blue = model.material("cart_blue", rgba=(0.18, 0.34, 0.62, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.28, 0.31, 0.34, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.71, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    def add_x_cylinder(
        part,
        *,
        radius: float,
        length: float,
        xyz: tuple[float, float, float],
        material,
        name: str | None = None,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    platform = model.part("platform")
    platform.inertial = Inertial.from_geometry(
        Box((platform_width, platform_length, deck_top_z)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, deck_top_z * 0.5)),
    )

    platform.visual(
        Box((platform_width - 0.04, platform_length - 0.04, deck_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_center_z)),
        material=deck_gray,
        name="deck_surface",
    )
    platform.visual(
        Box((0.03, platform_length - 0.04, frame_top_z - frame_bottom_z)),
        origin=Origin(xyz=(platform_width * 0.5 - 0.025, 0.0, frame_center_z)),
        material=cart_blue,
        name="right_side_frame",
    )
    platform.visual(
        Box((0.03, platform_length - 0.04, frame_top_z - frame_bottom_z)),
        origin=Origin(xyz=(-platform_width * 0.5 + 0.025, 0.0, frame_center_z)),
        material=cart_blue,
        name="left_side_frame",
    )
    platform.visual(
        Box((platform_width - 0.08, 0.03, frame_top_z - frame_bottom_z)),
        origin=Origin(xyz=(0.0, platform_length * 0.5 - 0.015, frame_center_z)),
        material=cart_blue,
        name="front_frame",
    )
    platform.visual(
        Box((platform_width - 0.08, 0.03, frame_top_z - frame_bottom_z)),
        origin=Origin(xyz=(0.0, -platform_length * 0.5 + 0.015, frame_center_z)),
        material=cart_blue,
        name="rear_frame",
    )
    platform.visual(
        Box((platform_width - 0.06, 0.06, 0.018)),
        origin=Origin(xyz=(0.0, 0.18, 0.137)),
        material=dark_steel,
        name="front_crossbrace",
    )
    platform.visual(
        Box((platform_width - 0.06, 0.06, 0.018)),
        origin=Origin(xyz=(0.0, -0.18, 0.137)),
        material=dark_steel,
        name="rear_crossbrace",
    )
    platform.visual(
        Box((0.06, platform_length - 0.18, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=dark_steel,
        name="center_spine",
    )

    caster_mounts = {
        "front_left": (-0.235, 0.385, frame_bottom_z),
        "front_right": (0.235, 0.385, frame_bottom_z),
        "rear_left": (-0.235, -0.385, frame_bottom_z),
        "rear_right": (0.235, -0.385, frame_bottom_z),
    }
    for mount_name, (x, y, _z) in caster_mounts.items():
        platform.visual(
            Box((0.10, 0.12, 0.010)),
            origin=Origin(xyz=(x, y, 0.133)),
            material=dark_steel,
            name=f"{mount_name}_caster_pad",
        )

    for side_x in (-0.20, 0.20):
        platform.visual(
            Box((0.04, 0.038, hinge_z - frame_bottom_z)),
            origin=Origin(xyz=(side_x, 0.457, (hinge_z + frame_bottom_z) * 0.5)),
            material=cart_blue,
            name=f"front_rail_mount_{'left' if side_x < 0 else 'right'}",
        )
    for side_x in (-0.23, 0.23):
        platform.visual(
            Box((0.05, 0.038, hinge_z - frame_bottom_z)),
            origin=Origin(xyz=(side_x, -0.457, (hinge_z + frame_bottom_z) * 0.5)),
            material=cart_blue,
            name=f"rear_handle_mount_{'left' if side_x < 0 else 'right'}",
        )

    front_rail = model.part("front_rail")
    front_rail.inertial = Inertial.from_geometry(
        Box((0.44, 0.05, 0.26)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.012, 0.12)),
    )
    front_rail.visual(
        Box((0.032, 0.024, 0.024)),
        origin=Origin(xyz=(-0.20, 0.012, 0.012)),
        material=steel,
        name="front_rail_left_foot",
    )
    front_rail.visual(
        Box((0.032, 0.024, 0.024)),
        origin=Origin(xyz=(0.20, 0.012, 0.012)),
        material=steel,
        name="front_rail_right_foot",
    )
    front_rail.visual(
        Cylinder(radius=0.014, length=0.24),
        origin=Origin(xyz=(-0.20, 0.020, 0.12)),
        material=steel,
        name="front_rail_left_post",
    )
    front_rail.visual(
        Cylinder(radius=0.014, length=0.24),
        origin=Origin(xyz=(0.20, 0.020, 0.12)),
        material=steel,
        name="front_rail_right_post",
    )
    add_x_cylinder(
        front_rail,
        radius=0.014,
        length=0.40,
        xyz=(0.0, 0.020, 0.24),
        material=steel,
        name="front_rail_top_bar",
    )
    add_x_cylinder(
        front_rail,
        radius=0.012,
        length=0.40,
        xyz=(0.0, 0.020, 0.11),
        material=steel,
        name="front_rail_mid_bar",
    )

    rear_handle = model.part("rear_handle")
    rear_handle.inertial = Inertial.from_geometry(
        Box((0.52, 0.06, 0.72)),
        mass=3.8,
        origin=Origin(xyz=(0.0, -0.014, 0.34)),
    )
    rear_handle.visual(
        Box((0.036, 0.028, 0.028)),
        origin=Origin(xyz=(-0.23, -0.014, 0.014)),
        material=steel,
        name="rear_handle_left_foot",
    )
    rear_handle.visual(
        Box((0.036, 0.028, 0.028)),
        origin=Origin(xyz=(0.23, -0.014, 0.014)),
        material=steel,
        name="rear_handle_right_foot",
    )
    rear_handle.visual(
        Cylinder(radius=0.016, length=0.68),
        origin=Origin(xyz=(-0.23, -0.024, 0.34)),
        material=steel,
        name="rear_handle_left_post",
    )
    rear_handle.visual(
        Cylinder(radius=0.016, length=0.68),
        origin=Origin(xyz=(0.23, -0.024, 0.34)),
        material=steel,
        name="rear_handle_right_post",
    )
    add_x_cylinder(
        rear_handle,
        radius=0.016,
        length=0.46,
        xyz=(0.0, -0.024, 0.68),
        material=steel,
        name="rear_handle_top_bar",
    )
    add_x_cylinder(
        rear_handle,
        radius=0.013,
        length=0.46,
        xyz=(0.0, -0.024, 0.28),
        material=steel,
        name="rear_handle_mid_bar",
    )

    def add_caster(prefix: str, mount_xyz: tuple[float, float, float]) -> None:
        fork_thickness = 0.014
        fork_inner_x = caster_width * 0.5
        fork_center_x = fork_inner_x + fork_thickness * 0.5

        caster = model.part(f"{prefix}_caster")
        caster.inertial = Inertial.from_geometry(
            Box((0.08, 0.15, 0.09)),
            mass=1.1,
            origin=Origin(xyz=(0.0, -0.055, -0.040)),
        )
        caster.visual(
            Box((0.070, 0.090, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=dark_steel,
            name="top_plate",
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, -0.016)),
            material=dark_steel,
            name="kingpin",
        )
        caster.visual(
            Box((0.048, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, 0.015, -0.027)),
            material=dark_steel,
            name="swivel_neck",
        )
        caster.visual(
            Box((0.062, 0.014, 0.014)),
            origin=Origin(xyz=(0.0, -0.135, -0.030)),
            material=dark_steel,
            name="yoke_bridge",
        )
        caster.visual(
            Box((fork_thickness, 0.144, 0.070)),
            origin=Origin(xyz=(-fork_center_x, -0.059, -0.051)),
            material=dark_steel,
            name="fork_left",
        )
        caster.visual(
            Box((fork_thickness, 0.144, 0.070)),
            origin=Origin(xyz=(fork_center_x, -0.059, -0.051)),
            material=dark_steel,
            name="fork_right",
        )

        wheel = model.part(f"{prefix}_wheel")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=caster_radius, length=caster_tire_width),
            mass=0.8,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )
        wheel.visual(
            Cylinder(radius=caster_radius, length=caster_tire_width),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=caster_hub_radius, length=caster_width),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="hub",
        )
        wheel.visual(
            Box((0.004, 0.006, 0.012)),
            origin=Origin(xyz=(caster_tire_width * 0.35, 0.0, caster_radius * 0.82)),
            material=steel,
            name="valve_stem",
        )

        model.articulation(
            f"platform_to_{prefix}_caster",
            ArticulationType.CONTINUOUS,
            parent=platform,
            child=caster,
            origin=Origin(xyz=mount_xyz),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=8.0),
        )
        model.articulation(
            f"{prefix}_caster_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, -caster_trail, -caster_swivel_drop)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=24.0),
        )

    model.articulation(
        "platform_to_front_rail",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=front_rail,
        origin=Origin(xyz=(0.0, 0.474, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.50),
    )
    model.articulation(
        "platform_to_rear_handle",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=rear_handle,
        origin=Origin(xyz=(0.0, -0.476, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    for prefix, mount_xyz in caster_mounts.items():
        add_caster(prefix, mount_xyz)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    platform = object_model.get_part("platform")
    front_rail = object_model.get_part("front_rail")
    rear_handle = object_model.get_part("rear_handle")

    front_rail_hinge = object_model.get_articulation("platform_to_front_rail")
    rear_handle_hinge = object_model.get_articulation("platform_to_rear_handle")

    front_left_caster = object_model.get_part("front_left_caster")
    front_right_caster = object_model.get_part("front_right_caster")
    rear_left_caster = object_model.get_part("rear_left_caster")
    rear_right_caster = object_model.get_part("rear_right_caster")

    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    front_left_swivel = object_model.get_articulation("platform_to_front_left_caster")
    front_right_swivel = object_model.get_articulation("platform_to_front_right_caster")
    rear_left_swivel = object_model.get_articulation("platform_to_rear_left_caster")
    rear_right_swivel = object_model.get_articulation("platform_to_rear_right_caster")

    front_left_spin = object_model.get_articulation("front_left_caster_to_wheel")
    front_right_spin = object_model.get_articulation("front_right_caster_to_wheel")
    rear_left_spin = object_model.get_articulation("rear_left_caster_to_wheel")
    rear_right_spin = object_model.get_articulation("rear_right_caster_to_wheel")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))

    def aabb_dims(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple(upper[index] - lower[index] for index in range(3))

    ctx.check(
        "front rail hinge axis is cross-cart",
        front_rail_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"axis={front_rail_hinge.axis}",
    )
    ctx.check(
        "rear handle hinge axis is cross-cart",
        rear_handle_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={rear_handle_hinge.axis}",
    )

    for name, swivel in (
        ("front left swivel", front_left_swivel),
        ("front right swivel", front_right_swivel),
        ("rear left swivel", rear_left_swivel),
        ("rear right swivel", rear_right_swivel),
    ):
        ctx.check(
            f"{name} uses vertical swivel axis",
            swivel.axis == (0.0, 0.0, 1.0),
            details=f"axis={swivel.axis}",
        )

    for name, spin in (
        ("front left wheel spin", front_left_spin),
        ("front right wheel spin", front_right_spin),
        ("rear left wheel spin", rear_left_spin),
        ("rear right wheel spin", rear_right_spin),
    ):
        ctx.check(
            f"{name} uses wheel axle axis",
            spin.axis == (1.0, 0.0, 0.0),
            details=f"axis={spin.axis}",
        )

    ctx.expect_contact(front_rail, platform, name="front rail is mounted to the platform")
    ctx.expect_contact(rear_handle, platform, name="rear handle is mounted to the platform")

    for caster_name, caster in (
        ("front left caster", front_left_caster),
        ("front right caster", front_right_caster),
        ("rear left caster", rear_left_caster),
        ("rear right caster", rear_right_caster),
    ):
        ctx.expect_contact(caster, platform, name=f"{caster_name} touches its platform mount")

    for wheel_name, wheel, caster in (
        ("front left wheel", front_left_wheel, front_left_caster),
        ("front right wheel", front_right_wheel, front_right_caster),
        ("rear left wheel", rear_left_wheel, rear_left_caster),
        ("rear right wheel", rear_right_wheel, rear_right_caster),
    ):
        ctx.expect_within(
            wheel,
            caster,
            axes="x",
            inner_elem="tire",
            margin=0.004,
            name=f"{wheel_name} stays between the fork cheeks",
        )
        ctx.expect_contact(
            wheel,
            caster,
            elem_a="hub",
            elem_b="fork_left",
            name=f"{wheel_name} hub touches the left fork cheek",
        )
        ctx.expect_contact(
            wheel,
            caster,
            elem_a="hub",
            elem_b="fork_right",
            name=f"{wheel_name} hub touches the right fork cheek",
        )

    front_top_rest = aabb_center(ctx.part_element_world_aabb(front_rail, elem="front_rail_top_bar"))
    with ctx.pose({front_rail_hinge: 1.45}):
        front_top_folded = aabb_center(ctx.part_element_world_aabb(front_rail, elem="front_rail_top_bar"))
    ctx.check(
        "front rail folds forward and downward",
        front_top_rest is not None
        and front_top_folded is not None
        and front_top_folded[1] > front_top_rest[1] + 0.16
        and front_top_folded[2] < front_top_rest[2] - 0.14,
        details=f"rest={front_top_rest}, folded={front_top_folded}",
    )

    rear_top_rest = aabb_center(ctx.part_element_world_aabb(rear_handle, elem="rear_handle_top_bar"))
    with ctx.pose({rear_handle_hinge: 1.35}):
        rear_top_folded = aabb_center(ctx.part_element_world_aabb(rear_handle, elem="rear_handle_top_bar"))
    ctx.check(
        "rear handle folds backward and downward",
        rear_top_rest is not None
        and rear_top_folded is not None
        and rear_top_folded[1] < rear_top_rest[1] - 0.30
        and rear_top_folded[2] < rear_top_rest[2] - 0.30,
        details=f"rest={rear_top_rest}, folded={rear_top_folded}",
    )

    front_left_wheel_dims_rest = aabb_dims(ctx.part_world_aabb(front_left_wheel))
    with ctx.pose({front_left_swivel: pi / 2.0}):
        front_left_wheel_dims_swiveled = aabb_dims(ctx.part_world_aabb(front_left_wheel))
    ctx.check(
        "front left caster swivel changes wheel footprint orientation",
        front_left_wheel_dims_rest is not None
        and front_left_wheel_dims_swiveled is not None
        and front_left_wheel_dims_rest[1] > front_left_wheel_dims_rest[0] * 2.0
        and front_left_wheel_dims_swiveled[0] > front_left_wheel_dims_swiveled[1] * 2.0,
        details=f"rest={front_left_wheel_dims_rest}, swiveled={front_left_wheel_dims_swiveled}",
    )

    valve_rest = aabb_center(ctx.part_element_world_aabb(front_left_wheel, elem="valve_stem"))
    with ctx.pose({front_left_spin: pi / 2.0}):
        valve_spun = aabb_center(ctx.part_element_world_aabb(front_left_wheel, elem="valve_stem"))
    ctx.check(
        "front left wheel spin moves the valve stem around the axle",
        valve_rest is not None
        and valve_spun is not None
        and (
            abs(valve_rest[1] - valve_spun[1]) > 0.025
            or abs(valve_rest[2] - valve_spun[2]) > 0.025
        ),
        details=f"rest={valve_rest}, spun={valve_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
