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
)


def _section_at_z(
    width: float,
    depth: float,
    corner_radius: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, corner_radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_juicer")

    body_grey = model.material("body_grey", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.19, 0.21, 0.23, 1.0))
    clear_lid = model.material("clear_lid", rgba=(0.82, 0.90, 0.96, 0.30))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    button_green = model.material("button_green", rgba=(0.26, 0.72, 0.40, 1.0))
    button_orange = model.material("button_orange", rgba=(0.90, 0.58, 0.20, 1.0))
    pusher_white = model.material("pusher_white", rgba=(0.97, 0.97, 0.95, 1.0))
    bin_grey = model.material("bin_grey", rgba=(0.36, 0.39, 0.42, 0.95))

    base = model.part("base")
    body_mesh = mesh_from_geometry(
        section_loft(
            [
                _section_at_z(0.255, 0.185, 0.056, 0.000),
                _section_at_z(0.268, 0.194, 0.060, 0.036),
                _section_at_z(0.240, 0.174, 0.054, 0.108),
                _section_at_z(0.188, 0.138, 0.042, 0.126),
            ]
        ),
        "base_shell",
    )
    base.visual(body_mesh, material=body_grey, name="body_shell")
    base.visual(
        Box((0.064, 0.084, 0.012)),
        origin=Origin(xyz=(-0.036, 0.000, 0.131)),
        material=body_grey,
        name="shoulder_pad",
    )
    base.visual(
        Cylinder(radius=0.066, length=0.016),
        origin=Origin(xyz=(0.046, 0.000, 0.128)),
        material=body_grey,
        name="top_collar",
    )
    base.visual(
        Box((0.060, 0.030, 0.016)),
        origin=Origin(xyz=(0.116, 0.000, 0.128)),
        material=dark_grey,
        name="spout_body",
    )
    base.visual(
        Box((0.026, 0.022, 0.010)),
        origin=Origin(xyz=(0.142, 0.000, 0.121)),
        material=dark_grey,
        name="spout_lip",
    )
    base.visual(
        Box((0.190, 0.104, 0.012)),
        origin=Origin(xyz=(-0.165, 0.000, 0.062)),
        material=dark_grey,
        name="rear_shelf",
    )
    for rail_y, rail_name in ((-0.043, "rear_rail_0"), (0.043, "rear_rail_1")):
        base.visual(
            Box((0.094, 0.008, 0.020)),
            origin=Origin(xyz=(-0.190, -0.052 if rail_name.endswith("0") else 0.052, 0.056)),
            material=dark_grey,
            name=rail_name,
        )
    for support_y, block_name, barrel_name in (
        (-0.045, "hinge_block_0", "hinge_barrel_0"),
        (0.045, "hinge_block_1", "hinge_barrel_1"),
    ):
        base.visual(
            Box((0.020, 0.018, 0.034)),
            origin=Origin(xyz=(-0.072, support_y, 0.153)),
            material=dark_grey,
            name=block_name,
        )
        base.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(
                xyz=(-0.072, support_y, 0.170),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_grey,
            name=barrel_name,
        )
    base.inertial = Inertial.from_geometry(
        Box((0.270, 0.195, 0.170)),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.000, 0.085)),
    )

    basket = model.part("basket")
    basket_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.000, 0.000),
                (0.018, 0.004),
                (0.046, 0.006),
                (0.050, 0.048),
            ],
            [
                (0.000, 0.004),
                (0.015, 0.010),
                (0.042, 0.010),
                (0.046, 0.046),
            ],
            segments=48,
        ),
        "basket_shell",
    )
    basket.visual(basket_mesh, material=steel, name="basket_shell")
    basket.visual(
        Cylinder(radius=0.035, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.008)),
        material=steel,
        name="cutter_disc",
    )
    basket.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=dark_grey,
        name="hub",
    )
    basket.visual(
        Box((0.062, 0.008, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=dark_grey,
        name="spoke_0",
    )
    basket.visual(
        Box((0.062, 0.008, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.010), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=dark_grey,
        name="spoke_1",
    )
    basket.inertial = Inertial.from_geometry(
        Box((0.104, 0.104, 0.052)),
        mass=0.28,
        origin=Origin(xyz=(0.000, 0.000, 0.026)),
    )

    lid = model.part("lid")
    lid_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.040, 0.050),
                (0.060, 0.046),
                (0.078, 0.032),
                (0.086, 0.016),
                (0.088, 0.004),
            ],
            [
                (0.036, 0.046),
                (0.056, 0.042),
                (0.074, 0.030),
                (0.082, 0.016),
                (0.084, 0.008),
            ],
            segments=56,
        ),
        "lid_cover",
    )
    lid.visual(
        lid_mesh,
        origin=Origin(xyz=(0.118, 0.000, -0.020)),
        material=clear_lid,
        name="cover_shell",
    )
    lid.visual(
        Box((0.036, 0.040, 0.018)),
        origin=Origin(xyz=(0.014, 0.000, -0.011)),
        material=clear_lid,
        name="rear_bridge",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.072),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.070, 0.004, 0.016)),
        origin=Origin(xyz=(0.118, -0.022, 0.038)),
        material=clear_lid,
        name="chute_skirt_0",
    )
    lid.visual(
        Box((0.070, 0.004, 0.016)),
        origin=Origin(xyz=(0.118, 0.022, 0.038)),
        material=clear_lid,
        name="chute_skirt_1",
    )
    lid.visual(
        Box((0.004, 0.040, 0.016)),
        origin=Origin(xyz=(0.083, 0.000, 0.038)),
        material=clear_lid,
        name="chute_skirt_2",
    )
    lid.visual(
        Box((0.004, 0.040, 0.016)),
        origin=Origin(xyz=(0.153, 0.000, 0.038)),
        material=clear_lid,
        name="chute_skirt_3",
    )
    lid.visual(
        Box((0.070, 0.004, 0.084)),
        origin=Origin(xyz=(0.118, -0.022, 0.088)),
        material=clear_lid,
        name="chute_wall_0",
    )
    lid.visual(
        Box((0.070, 0.004, 0.084)),
        origin=Origin(xyz=(0.118, 0.022, 0.088)),
        material=clear_lid,
        name="chute_wall_1",
    )
    lid.visual(
        Box((0.004, 0.040, 0.084)),
        origin=Origin(xyz=(0.083, 0.000, 0.088)),
        material=clear_lid,
        name="chute_wall_2",
    )
    lid.visual(
        Box((0.004, 0.040, 0.084)),
        origin=Origin(xyz=(0.153, 0.000, 0.088)),
        material=clear_lid,
        name="chute_wall_3",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.204, 0.156, 0.114)),
        mass=0.55,
        origin=Origin(xyz=(0.102, 0.000, 0.010)),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.058, 0.037, 0.150)),
        origin=Origin(xyz=(0.000, 0.000, -0.066)),
        material=pusher_white,
        name="shaft",
    )
    pusher.visual(
        Box((0.058, 0.003, 0.072)),
        origin=Origin(xyz=(0.000, -0.0185, -0.066)),
        material=pusher_white,
        name="guide_0",
    )
    pusher.visual(
        Box((0.058, 0.003, 0.072)),
        origin=Origin(xyz=(0.000, 0.0185, -0.066)),
        material=pusher_white,
        name="guide_1",
    )
    pusher.visual(
        Box((0.074, 0.052, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
        material=pusher_white,
        name="handle",
    )
    pusher.inertial = Inertial.from_geometry(
        Box((0.074, 0.052, 0.172)),
        mass=0.10,
        origin=Origin(xyz=(0.000, 0.000, -0.025)),
    )

    for index, button_y, material in (
        (0, -0.024, button_green),
        (1, 0.024, button_orange),
    ):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.004, length=0.014),
            origin=Origin(xyz=(0.000, 0.000, 0.000)),
            material=dark_grey,
            name="stem",
        )
        button.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.000, 0.000, 0.010)),
            material=material,
            name="cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.022, 0.022, 0.020)),
            mass=0.02,
            origin=Origin(xyz=(0.000, 0.000, 0.006)),
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(-0.024, button_y, 0.144)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0035,
            ),
        )

    bin_part = model.part("bin")
    bin_part.visual(
        Box((0.126, 0.092, 0.006)),
        origin=Origin(xyz=(-0.063, 0.000, 0.003)),
        material=bin_grey,
        name="bin_floor",
    )
    bin_part.visual(
        Box((0.118, 0.004, 0.072)),
        origin=Origin(xyz=(-0.063, -0.044, 0.039)),
        material=bin_grey,
        name="side_0",
    )
    bin_part.visual(
        Box((0.118, 0.004, 0.072)),
        origin=Origin(xyz=(-0.063, 0.044, 0.039)),
        material=bin_grey,
        name="side_1",
    )
    bin_part.visual(
        Box((0.004, 0.092, 0.072)),
        origin=Origin(xyz=(-0.124, 0.000, 0.039)),
        material=bin_grey,
        name="rear_wall",
    )
    bin_part.visual(
        Box((0.008, 0.092, 0.040)),
        origin=Origin(xyz=(-0.004, 0.000, 0.023)),
        material=bin_grey,
        name="front_wall",
    )
    bin_part.visual(
        Box((0.014, 0.050, 0.022)),
        origin=Origin(xyz=(-0.006, 0.000, 0.040)),
        material=bin_grey,
        name="handle_grip",
    )
    bin_part.inertial = Inertial.from_geometry(
        Box((0.140, 0.100, 0.078)),
        mass=0.32,
        origin=Origin(xyz=(-0.058, 0.000, 0.039)),
    )

    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.046, 0.000, 0.136)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.072, 0.000, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(75.0),
        ),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.118, 0.000, 0.154)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.018,
        ),
    )
    model.articulation(
        "base_to_bin",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bin_part,
        origin=Origin(xyz=(-0.135, 0.000, 0.068)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=0.090,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    bin_part = object_model.get_part("bin")
    button_0 = object_model.get_part("button_0")
    basket = object_model.get_part("basket")
    base = object_model.get_part("base")

    lid_joint = object_model.get_articulation("base_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    bin_joint = object_model.get_articulation("base_to_bin")
    button_joint = object_model.get_articulation("base_to_button_0")
    basket_joint = object_model.get_articulation("base_to_basket")

    ctx.allow_overlap(
        base,
        button_0,
        elem_b="stem",
        reason="The button stem intentionally plunges into the shoulder guide inside the base.",
    )
    ctx.allow_overlap(
        base,
        object_model.get_part("button_1"),
        elem_b="stem",
        reason="The button stem intentionally plunges into the shoulder guide inside the base.",
    )

    ctx.expect_overlap(
        bin_part,
        base,
        axes="x",
        elem_a="bin_floor",
        elem_b="rear_shelf",
        min_overlap=0.10,
        name="bin remains seated on rear shelf at rest",
    )
    ctx.expect_overlap(
        button_0,
        base,
        axes="xy",
        elem_a="cap",
        elem_b="shoulder_pad",
        min_overlap=0.016,
        name="button sits over shoulder pad",
    )

    lid_rest = ctx.part_element_world_aabb(lid, elem="cover_shell")
    pusher_rest = ctx.part_world_position(pusher)
    bin_rest = ctx.part_world_position(bin_part)
    button_rest = ctx.part_world_position(button_0)
    basket_rest = ctx.part_world_position(basket)

    with ctx.pose({lid_joint: math.radians(60.0)}):
        lid_open = ctx.part_element_world_aabb(lid, elem="cover_shell")
        ctx.check(
            "lid opens upward",
            lid_rest is not None
            and lid_open is not None
            and lid_open[1][2] > lid_rest[1][2] + 0.120,
            details=f"rest={lid_rest}, open={lid_open}",
        )

    with ctx.pose({pusher_joint: 0.018}):
        pusher_pressed = ctx.part_world_position(pusher)
        ctx.check(
            "pusher moves downward through chute",
            pusher_rest is not None
            and pusher_pressed is not None
            and pusher_pressed[2] < pusher_rest[2] - 0.010,
            details=f"rest={pusher_rest}, pressed={pusher_pressed}",
        )

    with ctx.pose({button_joint: 0.003}):
        button_pressed = ctx.part_world_position(button_0)
        ctx.check(
            "button plunges into shoulder",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[2] < button_rest[2] - 0.002,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    with ctx.pose({bin_joint: 0.090}):
        bin_open = ctx.part_world_position(bin_part)
        ctx.check(
            "bin slides rearward",
            bin_rest is not None
            and bin_open is not None
            and bin_open[0] < bin_rest[0] - 0.070,
            details=f"rest={bin_rest}, open={bin_open}",
        )
        ctx.expect_overlap(
            bin_part,
            base,
            axes="x",
            elem_a="bin_floor",
            elem_b="rear_shelf",
            min_overlap=0.030,
            name="bin retains shelf overlap when extended",
        )

    with ctx.pose({basket_joint: 1.4}):
        basket_spin = ctx.part_world_position(basket)
        ctx.check(
            "basket spins about a fixed vertical axis",
            basket_rest is not None
            and basket_spin is not None
            and abs(basket_spin[0] - basket_rest[0]) < 1e-6
            and abs(basket_spin[1] - basket_rest[1]) < 1e-6
            and abs(basket_spin[2] - basket_rest[2]) < 1e-6,
            details=f"rest={basket_rest}, spin={basket_spin}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
