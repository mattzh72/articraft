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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _xy_section(width_x: float, width_y: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width_x, width_y, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="centrifugal_juicer")

    body_silver = model.material("body_silver", rgba=(0.77, 0.79, 0.81, 1.0))
    body_dark = model.material("body_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    stainless = model.material("stainless", rgba=(0.83, 0.85, 0.87, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.78, 0.86, 0.92, 0.34))
    button_white = model.material("button_white", rgba=(0.95, 0.95, 0.97, 1.0))
    button_amber = model.material("button_amber", rgba=(0.78, 0.62, 0.20, 1.0))
    pusher_black = model.material("pusher_black", rgba=(0.11, 0.12, 0.13, 1.0))

    body = model.part("body")
    body_shell = section_loft(
        [
            _xy_section(0.248, 0.308, 0.042, 0.012),
            _xy_section(0.274, 0.326, 0.048, 0.120),
            _xy_section(0.228, 0.270, 0.040, 0.220),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell, "juicer_body_shell"),
        material=body_silver,
        name="shell",
    )
    body.visual(
        Box((0.252, 0.312, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_dark,
        name="base_foot",
    )
    body.visual(
        Cylinder(radius=0.096, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.216)),
        material=body_dark,
        name="chamber_seat",
    )
    body.visual(
        Box((0.022, 0.175, 0.090)),
        origin=Origin(xyz=(0.129, 0.0, 0.116)),
        material=body_dark,
        name="front_panel",
    )
    body.visual(
        Box((0.068, 0.160, 0.006)),
        origin=Origin(xyz=(0.137, 0.0, 0.160)),
        material=stainless,
        name="front_trim",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.280, 0.330, 0.230)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
    )

    chamber = model.part("chamber")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.020, 0.000),
            (0.082, 0.000),
            (0.100, 0.012),
            (0.108, 0.038),
            (0.106, 0.072),
        ],
        [
            (0.028, 0.005),
            (0.076, 0.006),
            (0.092, 0.016),
            (0.099, 0.040),
            (0.098, 0.072),
        ],
        segments=56,
    )
    chamber.visual(
        mesh_from_geometry(bowl_shell, "juicer_chamber_bowl"),
        material=clear_smoke,
        name="bowl_shell",
    )
    chamber.visual(
        Box((0.128, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.052, 0.006)),
        material=body_dark,
        name="base_ring_0",
    )
    chamber.visual(
        Box((0.128, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.052, 0.006)),
        material=body_dark,
        name="base_ring_1",
    )
    chamber.visual(
        Box((0.018, 0.086, 0.012)),
        origin=Origin(xyz=(-0.055, 0.0, 0.006)),
        material=body_dark,
        name="base_ring_2",
    )
    chamber.visual(
        Box((0.018, 0.086, 0.012)),
        origin=Origin(xyz=(0.055, 0.0, 0.006)),
        material=body_dark,
        name="base_ring_3",
    )
    chamber.visual(
        Box((0.030, 0.056, 0.030)),
        origin=Origin(xyz=(0.092, 0.0, 0.034)),
        material=body_dark,
        name="outlet_block",
    )
    chamber.visual(
        Box((0.028, 0.148, 0.014)),
        origin=Origin(xyz=(-0.122, 0.0, 0.069)),
        material=body_dark,
        name="hinge_bridge",
    )
    chamber.visual(
        Box((0.022, 0.148, 0.024)),
        origin=Origin(xyz=(-0.109, 0.0, 0.050)),
        material=body_dark,
        name="hinge_mount",
    )
    chamber.visual(
        Box((0.018, 0.024, 0.022)),
        origin=Origin(xyz=(-0.123, -0.072, 0.070)),
        material=body_dark,
        name="hinge_tab_0",
    )
    chamber.visual(
        Box((0.018, 0.024, 0.022)),
        origin=Origin(xyz=(-0.123, 0.072, 0.070)),
        material=body_dark,
        name="hinge_tab_1",
    )
    chamber.inertial = Inertial.from_geometry(
        Box((0.240, 0.240, 0.090)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )
    model.articulation(
        "body_to_chamber",
        ArticulationType.FIXED,
        parent=body,
        child=chamber,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
    )

    basket = model.part("basket")
    basket.visual(
        Cylinder(radius=0.072, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=stainless,
        name="basket_drum",
    )
    basket.visual(
        Cylinder(radius=0.084, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=stainless,
        name="cutter_disk",
    )
    basket.visual(
        Cylinder(radius=0.014, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=body_dark,
        name="drive_hub",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.084, length=0.062),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
    )
    model.articulation(
        "chamber_to_basket",
        ArticulationType.CONTINUOUS,
        parent=chamber,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=35.0),
    )

    lid = model.part("lid")
    lid_plate = ExtrudeWithHolesGeometry(
        superellipse_profile(0.224, 0.216, exponent=2.3, segments=56),
        [rounded_rect_profile(0.070, 0.052, 0.010, corner_segments=5)],
        0.006,
        center=False,
    )
    lid.visual(
        mesh_from_geometry(lid_plate, "juicer_lid_plate"),
        origin=Origin(xyz=(0.112, 0.0, 0.0)),
        material=clear_smoke,
        name="lid_plate",
    )
    lid.visual(
        Box((0.020, 0.156, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, 0.006)),
        material=clear_smoke,
        name="lid_bridge",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.118),
        origin=Origin(xyz=(0.000, 0.0, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="hinge_barrel",
    )

    chute_center_x = 0.112
    chute_center_z = 0.081
    chute_wall_height = 0.150
    lid.visual(
        Box((0.008, 0.068, chute_wall_height)),
        origin=Origin(xyz=(chute_center_x - 0.039, 0.0, chute_center_z)),
        material=clear_smoke,
        name="chute_rear_wall",
    )
    lid.visual(
        Box((0.008, 0.068, chute_wall_height)),
        origin=Origin(xyz=(chute_center_x + 0.039, 0.0, chute_center_z)),
        material=clear_smoke,
        name="chute_front_wall",
    )
    lid.visual(
        Box((0.086, 0.008, chute_wall_height)),
        origin=Origin(xyz=(chute_center_x, -0.030, chute_center_z)),
        material=clear_smoke,
        name="chute_side_0",
    )
    lid.visual(
        Box((0.086, 0.008, chute_wall_height)),
        origin=Origin(xyz=(chute_center_x, 0.030, chute_center_z)),
        material=clear_smoke,
        name="chute_side_1",
    )
    lid.visual(
        Box((0.012, 0.076, 0.008)),
        origin=Origin(xyz=(chute_center_x - 0.037, 0.0, 0.160)),
        material=body_dark,
        name="chute_trim_rear",
    )
    lid.visual(
        Box((0.012, 0.076, 0.008)),
        origin=Origin(xyz=(chute_center_x + 0.037, 0.0, 0.160)),
        material=body_dark,
        name="chute_trim_front",
    )
    lid.visual(
        Box((0.086, 0.012, 0.008)),
        origin=Origin(xyz=(chute_center_x, -0.032, 0.160)),
        material=body_dark,
        name="chute_trim_side_0",
    )
    lid.visual(
        Box((0.086, 0.012, 0.008)),
        origin=Origin(xyz=(chute_center_x, 0.032, 0.160)),
        material=body_dark,
        name="chute_trim_side_1",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.235, 0.220, 0.190)),
        mass=0.7,
        origin=Origin(xyz=(0.112, 0.0, 0.085)),
    )
    model.articulation(
        "chamber_to_lid",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=lid,
        origin=Origin(xyz=(-0.112, 0.0, 0.072)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.060, 0.042, 0.145)),
        origin=Origin(xyz=(0.0, 0.0, -0.0725)),
        material=pusher_black,
        name="plunger",
    )
    pusher.visual(
        Box((0.086, 0.066, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=pusher_black,
        name="cap",
    )
    pusher.visual(
        Box((0.060, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=pusher_black,
        name="handle",
    )
    pusher.inertial = Inertial.from_geometry(
        Box((0.086, 0.066, 0.171)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(chute_center_x, 0.0, 0.162)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.12,
            lower=0.0,
            upper=0.095,
        ),
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.0065, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="pivot_barrel",
    )
    spout.visual(
        Box((0.046, 0.030, 0.012)),
        origin=Origin(xyz=(0.025, 0.0, -0.006)),
        material=stainless,
        name="spout_body",
    )
    spout.visual(
        Box((0.012, 0.030, 0.008)),
        origin=Origin(xyz=(0.048, 0.0, -0.012)),
        material=stainless,
        name="mouth_lip",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.058, 0.032, 0.016)),
        mass=0.08,
        origin=Origin(xyz=(0.029, 0.0, -0.006)),
    )
    model.articulation(
        "chamber_to_spout",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=spout,
        origin=Origin(xyz=(0.1135, 0.0, 0.036)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(75.0),
        ),
    )

    for index, (y_pos, material_name) in enumerate(
        (
            (-0.028, button_white),
            (0.028, button_amber),
        )
    ):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.0105, length=0.003),
            origin=Origin(xyz=(0.0015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="button_bezel",
        )
        button.visual(
            Cylinder(radius=0.0090, length=0.007),
            origin=Origin(xyz=(0.0055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material_name,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0105, length=0.010),
            mass=0.03,
            origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.140, y_pos, 0.118)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.03,
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

    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    spout = object_model.get_part("spout")
    basket = object_model.get_part("basket")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    lid_hinge = object_model.get_articulation("chamber_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    spout_pivot = object_model.get_articulation("chamber_to_spout")
    button_0_slide = object_model.get_articulation("body_to_button_0")
    button_1_slide = object_model.get_articulation("body_to_button_1")
    basket_spin = object_model.get_articulation("chamber_to_basket")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            chamber,
            axis="z",
            positive_elem="lid_plate",
            negative_elem="bowl_shell",
            max_gap=0.008,
            max_penetration=0.0,
            name="lid seats on chamber rim",
        )
        ctx.expect_overlap(
            lid,
            chamber,
            axes="xy",
            elem_a="lid_plate",
            elem_b="bowl_shell",
            min_overlap=0.18,
            name="lid covers the chamber mouth",
        )
        ctx.expect_within(
            basket,
            chamber,
            axes="xy",
            inner_elem="basket_drum",
            outer_elem="bowl_shell",
            margin=0.0,
            name="basket stays centered within the chamber",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_plate")
    with ctx.pose({lid_hinge: math.radians(60.0)}):
        ctx.expect_gap(
            lid,
            chamber,
            axis="z",
            positive_elem="chute_front_wall",
            negative_elem="bowl_shell",
            min_gap=0.040,
            name="open lid lifts the chute above the chamber",
        )
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_plate")
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.085}):
        pusher_down = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides down the chute",
        pusher_rest is not None and pusher_down is not None and pusher_down[2] < pusher_rest[2] - 0.07,
        details=f"rest={pusher_rest}, down={pusher_down}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_0_slide: 0.0015, button_1_slide: 0.0015}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "front buttons press inward",
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_pressed is not None
        and button_1_pressed is not None
        and button_0_pressed[0] < button_0_rest[0] - 0.001
        and button_1_pressed[0] < button_1_rest[0] - 0.001,
        details=(
            f"button_0 rest={button_0_rest}, pressed={button_0_pressed}; "
            f"button_1 rest={button_1_rest}, pressed={button_1_pressed}"
        ),
    )

    spout_rest = ctx.part_element_world_aabb(spout, elem="mouth_lip")
    with ctx.pose({spout_pivot: math.radians(55.0)}):
        spout_up = ctx.part_element_world_aabb(spout, elem="mouth_lip")
    ctx.check(
        "spout flips upward",
        spout_rest is not None and spout_up is not None and spout_up[1][2] > spout_rest[1][2] + 0.02,
        details=f"rest={spout_rest}, up={spout_up}",
    )

    basket_limits = basket_spin.motion_limits
    ctx.check(
        "basket uses continuous spin limits",
        basket_limits is not None and basket_limits.lower is None and basket_limits.upper is None,
        details=f"limits={basket_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
