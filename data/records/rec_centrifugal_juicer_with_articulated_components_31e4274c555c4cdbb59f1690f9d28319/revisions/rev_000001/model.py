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


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_household_juicer")

    body_silver = model.material("body_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.82, 0.88, 0.91, 0.34))
    pusher_black = model.material("pusher_black", rgba=(0.18, 0.18, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))
    button_grey = model.material("button_grey", rgba=(0.72, 0.72, 0.74, 1.0))

    body = model.part("body")

    body_geom = section_loft(
        [
            _xy_section(0.230, 0.190, 0.040, 0.000),
            _xy_section(0.214, 0.176, 0.040, 0.055),
            _xy_section(0.190, 0.158, 0.038, 0.170),
            _xy_section(0.164, 0.144, 0.032, 0.300),
        ]
    )
    body.visual(
        mesh_from_geometry(body_geom, "body_shell"),
        material=body_silver,
        name="body_shell",
    )

    collar_geom = LatheGeometry.from_shell_profiles(
        [(0.092, 0.000), (0.092, 0.032)],
        [(0.082, 0.000), (0.082, 0.032)],
        segments=56,
    )
    body.visual(
        mesh_from_geometry(collar_geom, "top_collar"),
        origin=Origin(xyz=(0.000, 0.000, 0.300)),
        material=dark_trim,
        name="top_collar",
    )
    body.visual(
        Box((0.014, 0.090, 0.055)),
        origin=Origin(xyz=(0.099, 0.000, 0.140)),
        material=dark_trim,
        name="front_panel",
    )
    body.visual(
        Box((0.036, 0.082, 0.012)),
        origin=Origin(xyz=(-0.090, 0.000, 0.336)),
        material=dark_trim,
        name="hinge_block",
    )
    for index, y in enumerate((-0.023, 0.023)):
        body.visual(
            Cylinder(radius=0.007, length=0.016),
            origin=Origin(xyz=(-0.094, y, 0.346), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name=f"hinge_knuckle_{index}",
        )
    body.visual(
        Box((0.020, 0.014, 0.036)),
        origin=Origin(xyz=(0.062, 0.094, 0.350)),
        material=dark_trim,
        name="latch_mount",
    )
    body.visual(
        Box((0.024, 0.032, 0.064)),
        origin=Origin(xyz=(0.062, 0.079, 0.306)),
        material=dark_trim,
        name="latch_rib",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.230, 0.190, 0.332)),
        mass=5.6,
        origin=Origin(xyz=(0.000, 0.000, 0.166)),
    )

    lid = model.part("lid")

    lid_geom = LatheGeometry.from_shell_profiles(
        [
            (0.091, 0.000),
            (0.091, 0.012),
            (0.088, 0.052),
            (0.084, 0.070),
        ],
        [
            (0.084, 0.003),
            (0.084, 0.014),
            (0.081, 0.050),
            (0.077, 0.066),
        ],
        segments=64,
    )
    lid.visual(
        mesh_from_geometry(lid_geom, "lid_shell"),
        origin=Origin(xyz=(0.078, 0.000, 0.000)),
        material=smoked_clear,
        name="lid_shell",
    )

    chute_geom = LatheGeometry.from_shell_profiles(
        [(0.032, 0.000), (0.032, 0.128)],
        [(0.028, 0.000), (0.028, 0.128)],
        segments=40,
    )
    lid.visual(
        mesh_from_geometry(chute_geom, "chute_shell"),
        origin=Origin(xyz=(0.034, 0.044, 0.066)),
        material=smoked_clear,
        name="chute_shell",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.188, 0.184, 0.212)),
        mass=0.95,
        origin=Origin(xyz=(0.078, 0.000, 0.095)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.078, 0.000, 0.346)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    basket = model.part("basket")
    basket_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.056, 0.004),
            (0.066, 0.020),
            (0.073, 0.050),
            (0.076, 0.082),
        ],
        [
            (0.050, 0.006),
            (0.060, 0.022),
            (0.067, 0.050),
            (0.070, 0.078),
        ],
        segments=56,
    )
    basket.visual(
        mesh_from_geometry(basket_shell_geom, "basket_shell"),
        material=steel,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.054, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=steel,
        name="basket_base",
    )
    basket.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
        material=dark_trim,
        name="basket_spindle",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.076, length=0.086),
        mass=0.55,
        origin=Origin(xyz=(0.000, 0.000, 0.043)),
    )

    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.000, 0.000, 0.334)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=45.0),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.024, length=0.155),
        origin=Origin(xyz=(0.000, 0.000, -0.0475)),
        material=pusher_black,
        name="shaft",
    )
    pusher.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.000, 0.000, 0.045)),
        material=pusher_black,
        name="cap",
    )
    pusher.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(0.000, 0.000, 0.071)),
        material=pusher_black,
        name="grip",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.186),
        mass=0.24,
        origin=Origin(xyz=(0.000, 0.000, -0.004)),
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.034, 0.044, 0.194)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.085,
        ),
    )

    for index, z in enumerate((0.104, 0.140, 0.176)):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.003, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=button_grey,
            name="cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.008, 0.022, 0.022)),
            mass=0.03,
            origin=Origin(xyz=(0.003, 0.000, 0.000)),
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.106, 0.000, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="pivot",
    )
    latch.visual(
        Box((0.012, 0.010, 0.066)),
        origin=Origin(xyz=(0.000, 0.000, 0.033)),
        material=dark_trim,
        name="arm",
    )
    latch.visual(
        Box((0.012, 0.010, 0.014)),
        origin=Origin(xyz=(0.000, -0.001, 0.072)),
        material=dark_trim,
        name="hook",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.018, 0.016, 0.086)),
        mass=0.08,
        origin=Origin(xyz=(0.000, 0.000, 0.043)),
    )
    model.articulation(
        "body_to_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(0.062, 0.107, 0.350)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.6,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    latch = object_model.get_part("latch")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    latch_joint = object_model.get_articulation("body_to_latch")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="top_collar",
        min_gap=0.010,
        max_gap=0.030,
        name="lid shell sits just above the collar",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="top_collar",
        min_overlap=0.150,
        name="lid shell covers the body opening",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="shaft",
        outer_elem="chute_shell",
        margin=0.005,
        name="pusher shaft stays centered inside the chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="shaft",
        elem_b="chute_shell",
        min_overlap=0.110,
        name="pusher remains deeply inserted at rest",
    )

    with ctx.pose({pusher_slide: 0.085}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="shaft",
            outer_elem="chute_shell",
            margin=0.005,
            name="extended pusher stays centered in the chute",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="shaft",
            elem_b="chute_shell",
            min_overlap=0.025,
            name="extended pusher keeps retained insertion",
        )

    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: math.radians(72.0)}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward",
        lid_rest_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.080,
        details=f"rest={lid_rest_aabb}, open={lid_open_aabb}",
    )
    ctx.expect_origin_gap(
        basket,
        body,
        axis="z",
        min_gap=0.300,
        max_gap=0.360,
        name="filter basket is mounted high inside the upper chamber",
    )
    basket_joint = object_model.get_articulation("body_to_basket")
    ctx.check(
        "filter basket uses a vertical continuous spin joint",
        basket_joint.joint_type == ArticulationType.CONTINUOUS and basket_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={basket_joint.joint_type}, axis={basket_joint.axis}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.004}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[0] < rest_pos[0] - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    latch_rest = ctx.part_element_world_aabb(latch, elem="hook")
    with ctx.pose({latch_joint: 1.0}):
        latch_open = ctx.part_element_world_aabb(latch, elem="hook")
    ctx.check(
        "latch swings away from the lid edge",
        latch_rest is not None
        and latch_open is not None
        and latch_open[0][1] > latch_rest[0][1] + 0.018
        and latch_open[0][2] < latch_rest[0][2] - 0.020,
        details=f"closed={latch_rest}, open={latch_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
