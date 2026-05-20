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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_juicer")

    body_white = model.material("body_white", rgba=(0.93, 0.93, 0.91, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.24, 0.25, 0.28, 1.0))
    clear_lid = model.material("clear_lid", rgba=(0.83, 0.88, 0.92, 0.35))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))
    pusher_black = model.material("pusher_black", rgba=(0.10, 0.11, 0.12, 1.0))
    basket_metal = model.material("basket_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    power_green = model.material("power_green", rgba=(0.24, 0.60, 0.30, 1.0))
    pulse_orange = model.material("pulse_orange", rgba=(0.86, 0.50, 0.16, 1.0))

    base_height = 0.100
    chamber_center_y = 0.004
    chamber_origin_z = 0.089
    chamber_top_z = 0.141
    hinge_y = chamber_center_y + 0.082
    hinge_z = chamber_top_z + 0.001

    base_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.240, 0.180, 0.018),
            base_height,
            cap=True,
            closed=True,
        ),
        "juicer_base_shell",
    )
    chamber_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.083, 0.000),
                (0.083, 0.038),
                (0.079, 0.047),
                (0.075, 0.052),
            ],
            inner_profile=[
                (0.071, 0.002),
                (0.071, 0.036),
                (0.067, 0.044),
                (0.063, 0.049),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "juicer_chamber_shell",
    )
    lid_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.029, 0.055),
                (0.050, 0.054),
                (0.082, 0.042),
                (0.097, 0.020),
                (0.088, 0.000),
            ],
            inner_profile=[
                (0.024, 0.055),
                (0.044, 0.050),
                (0.075, 0.039),
                (0.090, 0.020),
                (0.074, 0.003),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "juicer_lid_shell",
    )
    chute_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.031, 0.000), (0.031, 0.050)],
            inner_profile=[(0.026, 0.002), (0.026, 0.048)],
            segments=40,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
        "juicer_chute",
    )
    chute_collar_mesh = mesh_from_geometry(
        TorusGeometry(0.0285, 0.0030, radial_segments=14, tubular_segments=32),
        "chute_collar_ring",
    )
    basket_ring_mesh = mesh_from_geometry(
        TorusGeometry(0.055, 0.0035, radial_segments=18, tubular_segments=40),
        "basket_ring",
    )

    body = model.part("body")
    body.visual(base_mesh, material=body_white, name="base_shell")
    body.visual(
        Box((0.112, 0.008, 0.044)),
        origin=Origin(xyz=(0.000, -0.088, 0.053)),
        material=panel_grey,
        name="control_panel",
    )
    body.visual(
        Box((0.040, 0.036, 0.022)),
        origin=Origin(xyz=(0.000, -0.093, 0.091)),
        material=body_white,
        name="spout_body",
    )
    body.visual(
        Box((0.030, 0.018, 0.010)),
        origin=Origin(xyz=(0.000, -0.118, 0.082)),
        material=body_white,
        name="spout_lip",
    )
    body.visual(
        chamber_mesh,
        origin=Origin(xyz=(0.000, chamber_center_y, chamber_origin_z)),
        material=panel_grey,
        name="chamber_shell",
    )
    body.visual(
        Cylinder(radius=0.074, length=0.006),
        origin=Origin(xyz=(0.000, chamber_center_y, 0.089)),
        material=dark_trim,
        name="chamber_floor",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.005),
        origin=Origin(xyz=(0.000, chamber_center_y, 0.0945)),
        material=dark_trim,
        name="drive_post",
    )
    body.visual(
        Box((0.118, 0.012, 0.014)),
        origin=Origin(xyz=(0.000, 0.080, 0.134)),
        material=dark_trim,
        name="hinge_bridge",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.240, 0.180, 0.150)),
        mass=3.6,
        origin=Origin(xyz=(0.000, 0.000, 0.075)),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.005, length=0.108),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )
    lid.visual(
        lid_shell_mesh,
        origin=Origin(xyz=(0.000, -0.082, 0.000)),
        material=clear_lid,
        name="lid_shell",
    )
    lid.visual(
        chute_mesh,
        origin=Origin(xyz=(0.000, -0.082, 0.055)),
        material=clear_lid,
        name="chute",
    )
    lid.visual(
        chute_collar_mesh,
        origin=Origin(xyz=(0.000, -0.082, 0.055)),
        material=clear_lid,
        name="chute_collar",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.200, 0.180, 0.110)),
        mass=0.55,
        origin=Origin(xyz=(0.000, -0.070, 0.050)),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.0225, length=0.104),
        origin=Origin(xyz=(0.000, 0.000, -0.052)),
        material=pusher_black,
        name="shaft",
    )
    pusher.visual(
        Cylinder(radius=0.0245, length=0.028),
        origin=Origin(xyz=(0.000, 0.000, -0.014)),
        material=pusher_black,
        name="guide_collar",
    )
    pusher.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=pusher_black,
        name="cap",
    )
    pusher.visual(
        Box((0.044, 0.034, 0.032)),
        origin=Origin(xyz=(0.000, 0.000, 0.022)),
        material=pusher_black,
        name="grip",
    )
    pusher.inertial = Inertial.from_geometry(
        Box((0.058, 0.058, 0.150)),
        mass=0.20,
        origin=Origin(xyz=(0.000, 0.000, -0.005)),
    )

    basket = model.part("basket")
    basket.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=basket_metal,
        name="hub",
    )
    basket.visual(
        Cylinder(radius=0.008, length=0.046),
        origin=Origin(xyz=(0.000, 0.000, 0.023)),
        material=basket_metal,
        name="spindle",
    )
    basket.visual(
        basket_ring_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=basket_metal,
        name="lower_ring",
    )
    basket.visual(
        basket_ring_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.042)),
        material=basket_metal,
        name="upper_ring",
    )
    basket.visual(
        Cylinder(radius=0.055, length=0.003),
        origin=Origin(xyz=(0.000, 0.000, 0.041)),
        material=basket_metal,
        name="top_disk",
    )
    for index in range(16):
        angle = 2.0 * math.pi * index / 16.0
        basket.visual(
            Cylinder(radius=0.0025, length=0.034),
            origin=Origin(
                xyz=(0.055 * math.cos(angle), 0.055 * math.sin(angle), 0.026),
            ),
            material=basket_metal,
            name=f"slat_{index}",
        )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.050),
        mass=0.35,
        origin=Origin(xyz=(0.000, 0.000, 0.025)),
    )

    button_0 = model.part("button_0")
    button_0.visual(
        Cylinder(radius=0.011, length=0.005),
        origin=Origin(xyz=(0.000, -0.0065, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=power_green,
        name="button_cap",
    )
    button_0.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.000, -0.0025, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="button_stem",
    )
    button_0.inertial = Inertial.from_geometry(
        Box((0.022, 0.014, 0.022)),
        mass=0.02,
        origin=Origin(xyz=(0.000, -0.005, 0.000)),
    )

    button_1 = model.part("button_1")
    button_1.visual(
        Cylinder(radius=0.011, length=0.005),
        origin=Origin(xyz=(0.000, -0.0065, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pulse_orange,
        name="button_cap",
    )
    button_1.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.000, -0.0025, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="button_stem",
    )
    button_1.inertial = Inertial.from_geometry(
        Box((0.022, 0.014, 0.022)),
        mass=0.02,
        origin=Origin(xyz=(0.000, -0.005, 0.000)),
    )

    lid_hinge = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.000, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.6,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.000, -0.082, 0.105)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.028,
        ),
    )
    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.000, chamber_center_y, 0.097)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=40.0),
    )
    model.articulation(
        "body_to_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_0,
        origin=Origin(xyz=(-0.026, -0.090, 0.053)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0015,
        ),
    )
    model.articulation(
        "body_to_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_1,
        origin=Origin(xyz=(0.026, -0.090, 0.053)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0015,
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

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    button_joint_0 = object_model.get_articulation("body_to_button_0")
    button_joint_1 = object_model.get_articulation("body_to_button_1")

    with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="chamber_shell",
            max_gap=0.004,
            max_penetration=0.0,
            name="lid rests just above chamber rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="chamber_shell",
            min_overlap=0.120,
            name="lid covers the chamber footprint",
        )
        ctx.expect_within(
            basket,
            body,
            axes="xy",
            inner_elem="top_disk",
            outer_elem="chamber_shell",
            margin=0.010,
            name="basket stays inside the chamber wall",
        )
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="shaft",
            outer_elem="chute",
            margin=0.002,
            name="pusher stays centered in the chute",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.040,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: pusher_slide.motion_limits.upper}):
        pressed_pusher_pos = ctx.part_world_position(pusher)
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="shaft",
            outer_elem="chute",
            margin=0.002,
            name="pusher remains centered at full stroke",
        )
    ctx.check(
        "pusher travels downward through the chute",
        rest_pusher_pos is not None
        and pressed_pusher_pos is not None
        and pressed_pusher_pos[2] < rest_pusher_pos[2] - 0.020,
        details=f"rest={rest_pusher_pos}, pressed={pressed_pusher_pos}",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose(
        {
            button_joint_0: button_joint_0.motion_limits.upper,
            button_joint_1: button_joint_1.motion_limits.upper,
        }
    ):
        pressed_button_0 = ctx.part_world_position(button_0)
        pressed_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "buttons press inward on the control panel",
        rest_button_0 is not None
        and rest_button_1 is not None
        and pressed_button_0 is not None
        and pressed_button_1 is not None
        and pressed_button_0[1] > rest_button_0[1] + 0.001
        and pressed_button_1[1] > rest_button_1[1] + 0.001,
        details=(
            f"button_0_rest={rest_button_0}, button_0_pressed={pressed_button_0}, "
            f"button_1_rest={rest_button_1}, button_1_pressed={pressed_button_1}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
