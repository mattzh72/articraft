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
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_juicer")

    body_white = model.material("body_white", rgba=(0.94, 0.94, 0.92, 1.0))
    smoky_clear = model.material("smoky_clear", rgba=(0.72, 0.80, 0.84, 0.28))
    steel = model.material("steel", rgba=(0.78, 0.79, 0.80, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    base = model.part("base")

    base_body = section_loft(
        [
            _xy_section(0.265, 0.215, 0.040, 0.040),
            _xy_section(0.335, 0.245, 0.070, 0.095),
            _xy_section(0.320, 0.235, 0.065, 0.165),
            _xy_section(0.250, 0.190, 0.045, 0.215),
        ]
    )
    base.visual(
        mesh_from_geometry(base_body, "base_body"),
        material=body_white,
        name="body_shell",
    )

    for index, (x_pos, y_pos) in enumerate(
        ((0.105, 0.078), (0.105, -0.078), (-0.095, 0.074), (-0.095, -0.074))
    ):
        base.visual(
            Cylinder(radius=0.013, length=0.040),
            origin=Origin(xyz=(x_pos, y_pos, 0.020)),
            material=rubber,
            name=f"foot_{index}",
        )

    chamber_outer = [(0.115, 0.000), (0.115, 0.050), (0.108, 0.085)]
    chamber_inner = [(0.097, 0.000), (0.097, 0.050), (0.091, 0.082)]
    chamber_shell = LatheGeometry.from_shell_profiles(
        chamber_outer,
        chamber_inner,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(
        mesh_from_geometry(chamber_shell, "chamber_ring"),
        origin=Origin(xyz=(-0.010, 0.000, 0.215)),
        material=body_white,
        name="chamber_ring",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(-0.010, 0.000, 0.215)),
        material=charcoal,
        name="drive_socket",
    )
    base.visual(
        Box((0.020, 0.072, 0.010)),
        origin=Origin(xyz=(-0.1055, 0.000, 0.295)),
        material=body_white,
        name="hinge_pad",
    )
    base.visual(
        Box((0.070, 0.040, 0.018)),
        origin=Origin(xyz=(0.135, 0.000, 0.252)),
        material=body_white,
        name="spout_body",
    )
    base.visual(
        Box((0.030, 0.032, 0.010)),
        origin=Origin(xyz=(0.183, 0.000, 0.242)),
        material=body_white,
        name="spout_lip",
    )
    base.visual(
        Box((0.094, 0.004, 0.070)),
        origin=Origin(xyz=(0.055, -0.121, 0.126)),
        material=charcoal,
        name="switch_panel",
    )
    base.visual(
        Box((0.120, 0.010, 0.004)),
        origin=Origin(xyz=(0.036, 0.045, 0.042)),
        material=charcoal,
        name="runner_plate_0",
    )
    base.visual(
        Box((0.120, 0.010, 0.004)),
        origin=Origin(xyz=(0.036, -0.045, 0.042)),
        material=charcoal,
        name="runner_plate_1",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.340, 0.250, 0.310)),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.000, 0.155)),
    )

    basket = model.part("basket")
    basket_outer = [(0.020, 0.000), (0.085, 0.000), (0.085, 0.062), (0.056, 0.078)]
    basket_inner = [(0.016, 0.004), (0.079, 0.004), (0.079, 0.059), (0.050, 0.074)]
    basket_shell = LatheGeometry.from_shell_profiles(
        basket_outer,
        basket_inner,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    basket.visual(
        mesh_from_geometry(basket_shell, "basket_shell"),
        material=steel,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.079, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.061)),
        material=steel,
        name="cutter_disk",
    )
    basket.visual(
        Cylinder(radius=0.020, length=0.065),
        origin=Origin(xyz=(0.000, 0.000, 0.0325)),
        material=charcoal,
        name="drive_hub",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.080),
        mass=0.55,
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
    )

    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(-0.010, 0.000, 0.220)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )

    lid = model.part("lid")
    lid_outer = [
        (0.105, 0.000),
        (0.101, 0.024),
        (0.082, 0.055),
        (0.060, 0.074),
        (0.048, 0.080),
    ]
    lid_inner = [
        (0.086, 0.003),
        (0.082, 0.025),
        (0.066, 0.053),
        (0.048, 0.071),
        (0.039, 0.078),
    ]
    lid_shell = LatheGeometry.from_shell_profiles(
        lid_outer,
        lid_inner,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    chute_outer = [(0.042, 0.000), (0.042, 0.118)]
    chute_inner = [(0.032, 0.000), (0.032, 0.118)]
    chute_shell = LatheGeometry.from_shell_profiles(
        chute_outer,
        chute_inner,
        segments=42,
        start_cap="flat",
        end_cap="flat",
    )
    chute_shell.translate(0.000, 0.000, 0.078)
    lid_shell.merge(chute_shell)
    lid.visual(
        mesh_from_geometry(lid_shell, "lid_shell"),
        origin=Origin(xyz=(0.105, 0.000, 0.000)),
        material=smoky_clear,
        name="lid_shell",
    )
    lid.visual(
        Box((0.014, 0.072, 0.010)),
        origin=Origin(xyz=(0.007, 0.000, 0.005)),
        material=smoky_clear,
        name="hinge_tongue",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.210, 0.210, 0.205)),
        mass=0.6,
        origin=Origin(xyz=(0.105, 0.000, 0.090)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.1155, 0.000, 0.300)),
        axis=(0.000, -1.000, 0.000),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.5,
            lower=0.000,
            upper=math.radians(72.0),
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.029, length=0.136),
        origin=Origin(xyz=(0.000, 0.000, -0.042)),
        material=charcoal,
        name="shaft",
    )
    pusher.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=charcoal,
        name="stop_flange",
    )
    pusher.visual(
        Cylinder(radius=0.037, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.019)),
        material=charcoal,
        name="top_grip",
    )
    pusher.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.037)),
        material=rubber,
        name="top_knob",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.164),
        mass=0.22,
        origin=Origin(xyz=(0.000, 0.000, -0.032)),
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.105, 0.000, 0.196)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.18,
            lower=0.000,
            upper=0.085,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.150, 0.116, 0.004)),
        origin=Origin(xyz=(0.075, 0.000, 0.004)),
        material=charcoal,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.130, 0.004, 0.020)),
        origin=Origin(xyz=(0.065, 0.056, 0.014)),
        material=charcoal,
        name="side_wall_0",
    )
    drawer.visual(
        Box((0.130, 0.004, 0.020)),
        origin=Origin(xyz=(0.065, -0.056, 0.014)),
        material=charcoal,
        name="side_wall_1",
    )
    drawer.visual(
        Box((0.006, 0.090, 0.016)),
        origin=Origin(xyz=(0.003, 0.000, 0.012)),
        material=charcoal,
        name="rear_wall",
    )
    drawer.visual(
        Box((0.014, 0.116, 0.028)),
        origin=Origin(xyz=(0.143, 0.000, 0.014)),
        material=body_white,
        name="front_face",
    )
    drawer.visual(
        Box((0.120, 0.010, 0.004)),
        origin=Origin(xyz=(0.060, 0.045, 0.038)),
        material=charcoal,
        name="runner_0",
    )
    drawer.visual(
        Box((0.120, 0.010, 0.004)),
        origin=Origin(xyz=(0.060, -0.045, 0.038)),
        material=charcoal,
        name="runner_1",
    )
    drawer.visual(
        Box((0.060, 0.010, 0.016)),
        origin=Origin(xyz=(0.065, 0.050, 0.030)),
        material=charcoal,
        name="runner_rib_0",
    )
    drawer.visual(
        Box((0.060, 0.010, 0.016)),
        origin=Origin(xyz=(0.065, -0.050, 0.030)),
        material=charcoal,
        name="runner_rib_1",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.156, 0.120, 0.040)),
        mass=0.28,
        origin=Origin(xyz=(0.078, 0.000, 0.020)),
    )

    model.articulation(
        "base_to_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(-0.020, 0.000, 0.002)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.20,
            lower=0.000,
            upper=0.100,
        ),
    )

    for name, x_pos in (("rear_switch", 0.022), ("front_switch", 0.078)):
        rocker = model.part(name)
        rocker.visual(
            Cylinder(radius=0.0045, length=0.022),
            origin=Origin(xyz=(0.000, -0.002, 0.015), rpy=(0.000, math.pi / 2.0, 0.000)),
            material=charcoal,
            name="pivot_barrel",
        )
        rocker.visual(
            Box((0.022, 0.012, 0.030)),
            origin=Origin(xyz=(0.000, -0.006, 0.015)),
            material=charcoal,
            name="paddle",
        )
        rocker.inertial = Inertial.from_geometry(
            Box((0.024, 0.014, 0.032)),
            mass=0.03,
            origin=Origin(xyz=(0.000, -0.006, 0.015)),
        )
        model.articulation(
            f"base_to_{name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=rocker,
            origin=Origin(xyz=(x_pos, -0.123, 0.111)),
            axis=(1.000, 0.000, 0.000),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=4.0,
                lower=math.radians(-12.0),
                upper=math.radians(12.0),
            ),
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

    base = object_model.get_part("base")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    drawer = object_model.get_part("drawer")
    front_switch = object_model.get_part("front_switch")
    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    drawer_slide = object_model.get_articulation("base_to_drawer")
    front_switch_joint = object_model.get_articulation("base_to_front_switch")

    ctx.expect_contact(
        lid,
        base,
        elem_a="hinge_tongue",
        elem_b="hinge_pad",
        name="lid stays mounted on rear hinge pad",
    )
    ctx.expect_within(
        basket,
        base,
        axes="xy",
        inner_elem="basket_shell",
        outer_elem="chamber_ring",
        margin=0.010,
        name="basket stays centered inside chamber",
    )
    ctx.expect_contact(
        pusher,
        lid,
        elem_a="stop_flange",
        elem_b="lid_shell",
        name="pusher rests on chute rim",
    )
    ctx.expect_contact(
        drawer,
        base,
        elem_a="runner_0",
        elem_b="runner_plate_0",
        name="drawer is supported by front runner",
    )
    ctx.expect_contact(
        front_switch,
        base,
        elem_a="paddle",
        elem_b="switch_panel",
        name="front switch sits on side panel",
    )
    with ctx.pose({lid_hinge: math.radians(55.0)}):
        ctx.expect_origin_gap(
            lid,
            base,
            axis="z",
            min_gap=0.030,
            name="lid rises when opened",
        )
    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.080}):
        ctx.expect_gap(
            pusher,
            lid,
            axis="z",
            min_gap=0.070,
            positive_elem="stop_flange",
            negative_elem="lid_shell",
            name="pusher lifts above chute rim",
        )
        pusher_raised = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides upward",
        pusher_rest is not None and pusher_raised is not None and pusher_raised[2] > pusher_rest[2] + 0.070,
        details=f"rest={pusher_rest}, raised={pusher_raised}",
    )
    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.095}):
        ctx.expect_overlap(
            drawer,
            base,
            axes="x",
            elem_a="runner_0",
            elem_b="runner_plate_0",
            min_overlap=0.010,
            name="drawer retains runner engagement when extended",
        )
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides forward",
        drawer_rest is not None and drawer_extended is not None and drawer_extended[0] > drawer_rest[0] + 0.090,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )
    switch_rest = ctx.part_element_world_aabb(front_switch, elem="paddle")
    with ctx.pose({front_switch_joint: math.radians(10.0)}):
        switch_tipped = ctx.part_element_world_aabb(front_switch, elem="paddle")
    ctx.check(
        "front switch rocks outward",
        switch_rest is not None
        and switch_tipped is not None
        and switch_tipped[0][1] < switch_rest[0][1] - 0.0015,
        details=f"rest={switch_rest}, tipped={switch_tipped}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
