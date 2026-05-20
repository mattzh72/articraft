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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_dial_candy_vending_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.74, 0.12, 0.10, 1.0))
    trim_black = model.material("trim_black", rgba=(0.12, 0.12, 0.13, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.20, 0.20, 0.22, 1.0))
    clear_acrylic = model.material("clear_acrylic", rgba=(0.84, 0.90, 0.96, 0.26))
    smoked_acrylic = model.material("smoked_acrylic", rgba=(0.30, 0.33, 0.36, 0.35))
    metal_silver = model.material("metal_silver", rgba=(0.78, 0.79, 0.82, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    candy_red = model.material("candy_red", rgba=(0.83, 0.20, 0.18, 1.0))
    candy_amber = model.material("candy_amber", rgba=(0.92, 0.62, 0.16, 1.0))
    candy_teal = model.material("candy_teal", rgba=(0.16, 0.66, 0.61, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    cabinet = model.part("cabinet")

    # Base and outer shell.
    cabinet.visual(
        Box((0.360, 0.280, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=trim_black,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.320, 0.008, 0.800)),
        origin=Origin(xyz=(0.0, -0.116, 0.450)),
        material=cabinet_red,
        name="rear_wall",
    )
    cabinet.visual(
        Box((0.012, 0.232, 0.800)),
        origin=Origin(xyz=(-0.164, 0.0, 0.450)),
        material=cabinet_red,
        name="left_wall",
    )
    cabinet.visual(
        Box((0.012, 0.232, 0.800)),
        origin=Origin(xyz=(0.164, 0.0, 0.450)),
        material=cabinet_red,
        name="right_wall",
    )
    cabinet.visual(
        Box((0.340, 0.240, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.865)),
        material=cabinet_red,
        name="top_cap",
    )

    # Front frame and dividers to create the tall three-bin facade.
    cabinet.visual(
        Box((0.040, 0.018, 0.800)),
        origin=Origin(xyz=(-0.150, 0.111, 0.450)),
        material=trim_black,
        name="front_left_pillar",
    )
    cabinet.visual(
        Box((0.040, 0.018, 0.800)),
        origin=Origin(xyz=(0.150, 0.111, 0.450)),
        material=trim_black,
        name="front_right_pillar",
    )
    cabinet.visual(
        Box((0.260, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, 0.111, 0.825)),
        material=trim_black,
        name="front_top_header",
    )
    cabinet.visual(
        Box((0.260, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, 0.111, 0.605)),
        material=trim_black,
        name="front_upper_divider",
    )
    cabinet.visual(
        Box((0.260, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, 0.111, 0.385)),
        material=trim_black,
        name="front_middle_divider",
    )
    cabinet.visual(
        Box((0.220, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.111, 0.197)),
        material=trim_black,
        name="door_lintel",
    )
    cabinet.visual(
        Box((0.058, 0.018, 0.152)),
        origin=Origin(xyz=(-0.131, 0.111, 0.126)),
        material=trim_black,
        name="door_left_jamb",
    )
    cabinet.visual(
        Box((0.058, 0.018, 0.152)),
        origin=Origin(xyz=(0.131, 0.111, 0.126)),
        material=trim_black,
        name="door_right_jamb",
    )
    cabinet.visual(
        Box((0.220, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.056, 0.058)),
        material=trim_dark,
        name="retrieval_floor",
    )
    cabinet.visual(
        Box((0.220, 0.110, 0.010)),
        origin=Origin(xyz=(0.0, 0.005, 0.194)),
        material=trim_dark,
        name="door_sill_shelf",
    )
    cabinet.visual(
        Box((0.028, 0.016, 0.028)),
        origin=Origin(xyz=(-0.070, 0.104, 0.065)),
        material=metal_silver,
        name="door_left_hinge_bracket",
    )
    cabinet.visual(
        Box((0.028, 0.016, 0.028)),
        origin=Origin(xyz=(0.070, 0.104, 0.065)),
        material=metal_silver,
        name="door_right_hinge_bracket",
    )
    cabinet.visual(
        Box((0.256, 0.016, 0.028)),
        origin=Origin(xyz=(0.0, 0.104, 0.065)),
        material=metal_silver,
        name="door_hinge_mount_rail",
    )

    # Internal shelves and hopper floors that visually support the candy.
    cabinet.visual(
        Box((0.316, 0.228, 0.012)),
        origin=Origin(xyz=(0.0, 0.004, 0.190)),
        material=trim_dark,
        name="lower_bin_floor",
    )
    cabinet.visual(
        Box((0.316, 0.228, 0.012)),
        origin=Origin(xyz=(0.0, 0.004, 0.410)),
        material=trim_dark,
        name="middle_bin_floor",
    )
    cabinet.visual(
        Box((0.316, 0.228, 0.012)),
        origin=Origin(xyz=(0.0, 0.004, 0.630)),
        material=trim_dark,
        name="upper_bin_floor",
    )
    for hopper_name, hopper_z in (
        ("upper_hopper", 0.660),
        ("middle_hopper", 0.440),
        ("lower_hopper", 0.220),
    ):
        cabinet.visual(
            Box((0.220, 0.120, 0.010)),
            origin=Origin(xyz=(0.0, 0.030, hopper_z), rpy=(-0.34, 0.0, 0.0)),
            material=trim_dark,
            name=hopper_name,
        )

    # Bin windows slightly overlap the frame so the whole cabinet reads as one
    # physically connected assembly.
    cabinet.visual(
        Box((0.272, 0.004, 0.182)),
        origin=Origin(xyz=(0.0, 0.118, 0.715)),
        material=clear_acrylic,
        name="upper_bin_window",
    )
    cabinet.visual(
        Box((0.272, 0.004, 0.182)),
        origin=Origin(xyz=(0.0, 0.118, 0.495)),
        material=clear_acrylic,
        name="middle_bin_window",
    )
    cabinet.visual(
        Box((0.272, 0.004, 0.162)),
        origin=Origin(xyz=(0.0, 0.118, 0.286)),
        material=clear_acrylic,
        name="lower_bin_window",
    )

    # Bezel collars behind the rotary selectors. Keep these names explicit
    # because run_tests() references them as exact-geometry anchors.
    cabinet.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.115, 0.710),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_silver,
        name="upper_selector_bezel",
    )
    cabinet.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.115, 0.490),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_silver,
        name="middle_selector_bezel",
    )
    cabinet.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.115, 0.270),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_silver,
        name="lower_selector_bezel",
    )

    # Candy fills: each cluster touches its supporting hopper/floor and is fused
    # by overlapping spheres so it does not read as floating debris.
    def add_candy_fill(prefix: str, base_z: float, material) -> None:
        cabinet.visual(
            Box((0.220, 0.095, 0.050)),
            origin=Origin(xyz=(0.0, 0.024, base_z)),
            material=material,
            name=f"{prefix}_candy_base",
        )
        for index, (x, y, z, radius) in enumerate(
            (
                (-0.072, 0.018, base_z + 0.036, 0.030),
                (-0.018, 0.010, base_z + 0.046, 0.032),
                (0.040, 0.022, base_z + 0.041, 0.029),
                (0.086, 0.012, base_z + 0.034, 0.026),
            )
        ):
            cabinet.visual(
                Sphere(radius=radius),
                origin=Origin(xyz=(x, y, z)),
                material=material,
                name=f"{prefix}_candy_bulge_{index}",
            )

    add_candy_fill("upper", 0.661, candy_red)
    add_candy_fill("middle", 0.441, candy_amber)
    add_candy_fill("lower", 0.221, candy_teal)

    # Feet under the base.
    for name, x, y in (
        ("foot_front_left", -0.120, 0.082),
        ("foot_front_right", 0.120, 0.082),
        ("foot_back_left", -0.120, -0.082),
        ("foot_back_right", 0.120, -0.082),
    ):
        cabinet.visual(
            Box((0.028, 0.028, 0.012)),
            origin=Origin(xyz=(x, y, -0.006)),
            material=foot_rubber,
            name=name,
        )

    cabinet.inertial = Inertial.from_geometry(
        Box((0.360, 0.280, 0.880)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
    )

    selector_specs = (
        ("upper_selector", 0.710),
        ("middle_selector", 0.490),
        ("lower_selector", 0.270),
    )
    for selector_name, selector_z in selector_specs:
        selector = model.part(selector_name)
        selector.visual(
            Cylinder(radius=0.034, length=0.032),
            origin=Origin(
                xyz=(0.0, 0.016, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=knob_black,
            name="selector_body",
        )
        selector.visual(
            Cylinder(radius=0.043, length=0.008),
            origin=Origin(
                xyz=(0.0, 0.004, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_silver,
            name="selector_flange",
        )
        selector.visual(
            Box((0.018, 0.012, 0.054)),
            origin=Origin(xyz=(0.0, 0.035, 0.0)),
            material=metal_silver,
            name="selector_grip",
        )
        selector.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(
                xyz=(0.0, 0.038, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_silver,
            name="selector_cap",
        )
        selector.inertial = Inertial.from_geometry(
            Cylinder(radius=0.043, length=0.040),
            mass=0.22,
            origin=Origin(
                xyz=(0.0, 0.020, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
        )
        model.articulation(
            f"cabinet_to_{selector_name}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=selector,
            origin=Origin(xyz=(0.0, 0.120, selector_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=8.0),
        )

    door = model.part("retrieval_door")
    door.visual(
        Box((0.200, 0.016, 0.120)),
        origin=Origin(xyz=(0.0, 0.008, 0.060)),
        material=trim_black,
        name="door_shell",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_silver,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.146, 0.006, 0.082)),
        origin=Origin(xyz=(0.0, 0.016, 0.068)),
        material=smoked_acrylic,
        name="door_window",
    )
    door.visual(
        Box((0.084, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, 0.022, 0.098)),
        material=metal_silver,
        name="door_pull",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.200, 0.016, 0.120)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.008, 0.060)),
    )
    model.articulation(
        "cabinet_to_retrieval_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, 0.120, 0.065)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    upper_selector = object_model.get_part("upper_selector")
    middle_selector = object_model.get_part("middle_selector")
    lower_selector = object_model.get_part("lower_selector")
    retrieval_door = object_model.get_part("retrieval_door")

    upper_joint = object_model.get_articulation("cabinet_to_upper_selector")
    middle_joint = object_model.get_articulation("cabinet_to_middle_selector")
    lower_joint = object_model.get_articulation("cabinet_to_lower_selector")
    door_joint = object_model.get_articulation("cabinet_to_retrieval_door")

    for name, joint in (
        ("upper", upper_joint),
        ("middle", middle_joint),
        ("lower", lower_joint),
    ):
        ctx.check(
            f"{name} selector uses continuous rotary articulation",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{name} selector rotates on horizontal shaft",
            abs(joint.axis[1]) > 0.99 and abs(joint.axis[0]) < 1e-6 and abs(joint.axis[2]) < 1e-6,
            details=f"axis={joint.axis}",
        )

    ctx.expect_origin_gap(
        upper_selector,
        middle_selector,
        axis="z",
        min_gap=0.18,
        max_gap=0.26,
        name="upper selector sits above middle selector",
    )
    ctx.expect_origin_gap(
        middle_selector,
        lower_selector,
        axis="z",
        min_gap=0.18,
        max_gap=0.26,
        name="middle selector sits above lower selector",
    )

    ctx.expect_gap(
        upper_selector,
        cabinet,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem="selector_flange",
        negative_elem="upper_selector_bezel",
        name="upper selector seats against bezel",
    )
    ctx.expect_gap(
        middle_selector,
        cabinet,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem="selector_flange",
        negative_elem="middle_selector_bezel",
        name="middle selector seats against bezel",
    )
    ctx.expect_gap(
        lower_selector,
        cabinet,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem="selector_flange",
        negative_elem="lower_selector_bezel",
        name="lower selector seats against bezel",
    )

    ctx.check(
        "retrieval door uses a bottom hinge",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(door_joint.axis[0]) > 0.99
        and abs(door_joint.axis[1]) < 1e-6
        and abs(door_joint.axis[2]) < 1e-6,
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_contact(
            retrieval_door,
            cabinet,
            elem_a="door_hinge_barrel",
            elem_b="door_left_hinge_bracket",
            name="door hinge barrel contacts left hinge bracket",
        )
        ctx.expect_gap(
            retrieval_door,
            cabinet,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="door_shell",
            negative_elem="door_left_jamb",
            name="door sits flush with front frame plane",
        )
        closed_pull_aabb = ctx.part_element_world_aabb(retrieval_door, elem="door_pull")

    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        open_pull_aabb = ctx.part_element_world_aabb(retrieval_door, elem="door_pull")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))

    closed_pull_center = _aabb_center(closed_pull_aabb)
    open_pull_center = _aabb_center(open_pull_aabb)
    ctx.check(
        "retrieval door opens downward and outward",
        closed_pull_center is not None
        and open_pull_center is not None
        and open_pull_center[1] > closed_pull_center[1] + 0.05
        and open_pull_center[2] < closed_pull_center[2] - 0.03,
        details=f"closed={closed_pull_center}, open={open_pull_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
