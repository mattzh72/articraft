from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BRONZE = "oil_rubbed_bronze"
GLASS = "slightly_smoked_glass"
WARM_GLASS = "warm_lamp_glass"


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").cylinder(length, radius).translate(center)


def _union(shapes: list[cq.Workplane]) -> cq.Workplane:
    solid = shapes[0]
    for shape in shapes[1:]:
        solid = solid.union(shape)
    return solid


def _housing_mesh() -> cq.Workplane:
    width = 0.40
    depth = 0.30
    post = 0.026
    rail = 0.032
    lower_z = 0.065
    upper_z = 0.550
    rail_h = 0.042
    post_h = upper_z - lower_z + rail_h
    post_z = (upper_z + lower_z) / 2.0

    shapes: list[cq.Workplane] = []

    # Four cast corner posts tie the upper and lower rectangular rail frames together.
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            shapes.append(
                _cq_box(
                    (post, post, post_h),
                    (sx * (width / 2.0 - post / 2.0), sy * (depth / 2.0 - post / 2.0), post_z),
                )
            )

    # Lower and upper rectangular cast rails around the lantern body.
    for z in (lower_z, upper_z):
        shapes.extend(
            [
                _cq_box((width, rail, rail_h), (0.0, depth / 2.0 - rail / 2.0, z)),
                _cq_box((width, rail, rail_h), (0.0, -depth / 2.0 + rail / 2.0, z)),
                _cq_box((rail, depth, rail_h), (width / 2.0 - rail / 2.0, 0.0, z)),
                _cq_box((rail, depth, rail_h), (-width / 2.0 + rail / 2.0, 0.0, z)),
            ]
        )

    # Subtle raised beads on the top and bottom cast rails.
    bead = 0.008
    for z in (lower_z + 0.025, upper_z - 0.025):
        shapes.extend(
            [
                _cq_box((width + 0.010, bead, bead), (0.0, depth / 2.0 + bead / 2.0, z)),
                _cq_box((width + 0.010, bead, bead), (0.0, -depth / 2.0 - bead / 2.0, z)),
                _cq_box((bead, depth + 0.010, bead), (width / 2.0 + bead / 2.0, 0.0, z)),
                _cq_box((bead, depth + 0.010, bead), (-width / 2.0 - bead / 2.0, 0.0, z)),
            ]
        )

    # Ceiling-mount roof stack and canopy, kept physically connected to the top rail.
    shapes.extend(
        [
            _cq_box((0.46, 0.36, 0.042), (0.0, 0.0, 0.585)),
            _cq_box((0.34, 0.24, 0.034), (0.0, 0.0, 0.625)),
            _cq_cylinder(0.055, 0.034, (0.0, 0.0, 0.645)),
            _cq_cylinder(0.026, 0.128, (0.0, 0.0, 0.705)),
            _cq_cylinder(0.130, 0.036, (0.0, 0.0, 0.775)),
        ]
    )

    # A small bottom finial makes the hanging carriage-lantern silhouette recognizable.
    shapes.extend(
        [
            _cq_cylinder(0.022, 0.040, (0.0, 0.0, 0.022)),
            _cq_cylinder(0.012, 0.030, (0.0, 0.0, -0.010)),
        ]
    )

    return _union(shapes)


def _add_housing_visuals(housing) -> None:
    width = 0.40
    depth = 0.30
    post = 0.026
    rail = 0.032
    lower_z = 0.065
    upper_z = 0.550
    rail_h = 0.042
    post_h = upper_z - lower_z + rail_h + 0.008
    post_z = (upper_z + lower_z) / 2.0

    for ix, sx in enumerate((-1.0, 1.0)):
        for iy, sy in enumerate((-1.0, 1.0)):
            housing.visual(
                Box((post, post, post_h)),
                origin=Origin(
                    xyz=(
                        sx * (width / 2.0 - post / 2.0),
                        sy * (depth / 2.0 - post / 2.0),
                        post_z,
                    )
                ),
                material=BRONZE,
                name=f"corner_post_{ix}_{iy}",
            )

    for label, z in (("lower", lower_z), ("upper", upper_z)):
        housing.visual(
            Box((width, rail, rail_h)),
            origin=Origin(xyz=(0.0, depth / 2.0 - rail / 2.0, z)),
            material=BRONZE,
            name=f"{label}_front_rail",
        )
        housing.visual(
            Box((width, rail, rail_h)),
            origin=Origin(xyz=(0.0, -depth / 2.0 + rail / 2.0, z)),
            material=BRONZE,
            name=f"{label}_rear_rail",
        )
        housing.visual(
            Box((rail, depth, rail_h)),
            origin=Origin(xyz=(width / 2.0 - rail / 2.0, 0.0, z)),
            material=BRONZE,
            name=f"{label}_side_rail_0",
        )
        housing.visual(
            Box((rail, depth, rail_h)),
            origin=Origin(xyz=(-width / 2.0 + rail / 2.0, 0.0, z)),
            material=BRONZE,
            name=f"{label}_side_rail_1",
        )

    bead = 0.008
    for label, z in (("lower", lower_z + 0.025), ("upper", upper_z - 0.025)):
        housing.visual(
            Box((width + 0.010, bead, bead)),
            origin=Origin(xyz=(0.0, depth / 2.0 + bead / 2.0, z)),
            material=BRONZE,
            name=f"{label}_front_bead",
        )
        housing.visual(
            Box((width + 0.010, bead, bead)),
            origin=Origin(xyz=(0.0, -depth / 2.0 - bead / 2.0, z)),
            material=BRONZE,
            name=f"{label}_rear_bead",
        )
        housing.visual(
            Box((bead, depth + 0.010, bead)),
            origin=Origin(xyz=(width / 2.0 + bead / 2.0, 0.0, z)),
            material=BRONZE,
            name=f"{label}_side_bead_0",
        )
        housing.visual(
            Box((bead, depth + 0.010, bead)),
            origin=Origin(xyz=(-width / 2.0 - bead / 2.0, 0.0, z)),
            material=BRONZE,
            name=f"{label}_side_bead_1",
        )

    housing.visual(
        Box((0.46, 0.36, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=BRONZE,
        name="roof_plate",
    )
    housing.visual(
        Box((0.34, 0.24, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=BRONZE,
        name="ceiling_boss",
    )
    housing.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        material=BRONZE,
        name="round_collar",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.710)),
        material=BRONZE,
        name="ceiling_stem",
    )
    housing.visual(
        Cylinder(radius=0.130, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=BRONZE,
        name="ceiling_canopy",
    )
    housing.visual(
        Box((0.36, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=BRONZE,
        name="bottom_crossbar_x",
    )
    housing.visual(
        Box((0.020, 0.26, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=BRONZE,
        name="bottom_crossbar_y",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=BRONZE,
        name="bottom_finial",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=BRONZE,
        name="finial_tip",
    )


def _door_frame_mesh(width: float, height: float) -> cq.Workplane:
    strip = 0.018
    depth = 0.014
    mullion = 0.010
    hinge_r = 0.007
    hinge_len = 0.083
    shapes = [
        _cq_box((strip, depth, height), (strip / 2.0, 0.0, 0.0)),
        _cq_box((strip, depth, height), (width - strip / 2.0, 0.0, 0.0)),
        _cq_box((width, depth, strip), (width / 2.0, 0.0, height / 2.0 - strip / 2.0)),
        _cq_box((width, depth, strip), (width / 2.0, 0.0, -height / 2.0 + strip / 2.0)),
        _cq_box((width - 2.0 * strip, depth * 0.9, mullion), (width / 2.0, 0.0, 0.0)),
    ]

    # Three exposed knuckles on the hinged vertical edge.
    for z in (-0.145, 0.0, 0.145):
        shapes.append(_cq_cylinder(hinge_r, hinge_len, (0.0, 0.0, z)))

    # A small latch knob on the free vertical edge, mounted on the outside face.
    shapes.append(_cq_cylinder(0.009, 0.014, (width - 0.010, 0.014, 0.0)))

    return _union(shapes)


def _add_glass_door(
    model: ArticulatedObject,
    parent,
    *,
    name: str,
    width: float,
    height: float,
    joint_xyz: tuple[float, float, float],
    yaw: float,
) -> None:
    door = model.part(name)
    door.visual(
        mesh_from_cadquery(_door_frame_mesh(width, height), f"{name}_cast_frame"),
        material=BRONZE,
        name="cast_frame",
    )
    door.visual(
        Box((width - 0.024, 0.004, height - 0.052)),
        origin=Origin(xyz=(width / 2.0, 0.0, 0.0)),
        material=GLASS,
        name="glass_pane",
    )

    model.articulation(
        f"housing_to_{name}",
        ArticulationType.REVOLUTE,
        parent=parent,
        child=door,
        origin=Origin(xyz=joint_xyz, rpy=(0.0, 0.0, yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=4.0, velocity=1.2),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_carriage_lantern")
    model.material(BRONZE, rgba=(0.055, 0.045, 0.035, 1.0))
    model.material(GLASS, rgba=(0.65, 0.86, 0.92, 0.34))
    model.material(WARM_GLASS, rgba=(1.0, 0.74, 0.36, 0.72))

    housing = model.part("housing")
    _add_housing_visuals(housing)
    housing.visual(
        Cylinder(radius=0.023, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        material=BRONZE,
        name="lamp_socket",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=WARM_GLASS,
        name="warm_candle",
    )
    housing.visual(
        Sphere(radius=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=WARM_GLASS,
        name="warm_bulb",
    )

    door_height = 0.430
    center_z = 0.320
    front_width = 0.330
    side_width = 0.230
    front_y = 0.165
    side_x = 0.215

    _add_glass_door(
        model,
        housing,
        name="front_door",
        width=front_width,
        height=door_height,
        joint_xyz=(-front_width / 2.0, front_y, center_z),
        yaw=0.0,
    )
    _add_glass_door(
        model,
        housing,
        name="rear_door",
        width=front_width,
        height=door_height,
        joint_xyz=(front_width / 2.0, -front_y, center_z),
        yaw=math.pi,
    )
    _add_glass_door(
        model,
        housing,
        name="side_door_0",
        width=side_width,
        height=door_height,
        joint_xyz=(side_x, side_width / 2.0, center_z),
        yaw=-math.pi / 2.0,
    )
    _add_glass_door(
        model,
        housing,
        name="side_door_1",
        width=side_width,
        height=door_height,
        joint_xyz=(-side_x, -side_width / 2.0, center_z),
        yaw=math.pi / 2.0,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door_names = ("front_door", "rear_door", "side_door_0", "side_door_1")
    hinge_names = tuple(f"housing_to_{door}" for door in door_names)

    ctx.check(
        "four separate hinged glass doors",
        all(object_model.get_part(name) is not None for name in door_names)
        and all(object_model.get_articulation(name) is not None for name in hinge_names),
        details=f"doors={door_names}, hinges={hinge_names}",
    )

    for hinge_name in hinge_names:
        hinge = object_model.get_articulation(hinge_name)
        ctx.check(
            f"{hinge_name} is a vertical revolute hinge",
            hinge.articulation_type == ArticulationType.REVOLUTE and tuple(hinge.axis) == (0.0, 0.0, 1.0),
            details=f"type={hinge.articulation_type}, axis={hinge.axis}",
        )

    # Closed panels sit just outside the cast frame but share its vertical opening.
    housing = object_model.get_part("housing")
    for door_name in door_names:
        ctx.expect_overlap(
            door_name,
            housing,
            axes="z",
            min_overlap=0.35,
            elem_a="glass_pane",
            name=f"{door_name} covers the lantern opening vertically",
        )

    def element_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    # Positive joint travel swings each panel outward from the lantern body.
    for door_name, outward_axis, sign in (
        ("front_door", 1, 1.0),
        ("rear_door", 1, -1.0),
        ("side_door_0", 0, 1.0),
        ("side_door_1", 0, -1.0),
    ):
        hinge = object_model.get_articulation(f"housing_to_{door_name}")
        closed_pos = element_center(door_name, "glass_pane")
        with ctx.pose({hinge: 0.85}):
            opened_pos = element_center(door_name, "glass_pane")
        ctx.check(
            f"{door_name} opens outward",
            closed_pos is not None
            and opened_pos is not None
            and (opened_pos[outward_axis] - closed_pos[outward_axis]) * sign > 0.040,
            details=f"closed={closed_pos}, opened={opened_pos}",
        )

    return ctx.report()


object_model = build_object_model()
