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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _prism_from_profile(
    profile_yz: list[tuple[float, float]],
    *,
    x_min: float,
    x_max: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    left_ids: list[int] = []
    right_ids: list[int] = []

    for y, z in profile_yz:
        left_ids.append(geom.add_vertex(x_min, y, z))
        right_ids.append(geom.add_vertex(x_max, y, z))

    count = len(profile_yz)
    for index in range(count):
        next_index = (index + 1) % count
        _add_quad(
            geom,
            left_ids[index],
            left_ids[next_index],
            right_ids[next_index],
            right_ids[index],
        )

    for index in range(1, count - 1):
        geom.add_face(left_ids[0], left_ids[index + 1], left_ids[index])
        geom.add_face(right_ids[0], right_ids[index], right_ids[index + 1])

    return geom


def _segment_pose(
    start_yz: tuple[float, float],
    end_yz: tuple[float, float],
) -> tuple[float, float, float, float]:
    start_y, start_z = start_yz
    end_y, end_z = end_yz
    delta_y = end_y - start_y
    delta_z = end_z - start_z
    return (
        (start_y + end_y) * 0.5,
        (start_z + end_z) * 0.5,
        math.hypot(delta_y, delta_z),
        math.atan2(delta_z, delta_y),
    )


def _offset_along_local_z(y: float, z: float, angle: float, offset: float) -> tuple[float, float]:
    return (y - math.sin(angle) * offset, z + math.cos(angle) * offset)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bartop_arcade_machine")

    body_black = model.material("body_black", rgba=(0.10, 0.10, 0.11, 1.0))
    trim_black = model.material("trim_black", rgba=(0.04, 0.04, 0.05, 1.0))
    control_black = model.material("control_black", rgba=(0.09, 0.09, 0.10, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.09, 0.16, 0.20, 0.92))
    marquee_glow = model.material("marquee_glow", rgba=(0.76, 0.18, 0.84, 0.82))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    knob_indicator = model.material("knob_indicator", rgba=(0.82, 0.82, 0.83, 1.0))
    coin_metal = model.material("coin_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    coin_dark = model.material("coin_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    button_red = model.material("button_red", rgba=(0.72, 0.14, 0.16, 1.0))
    button_yellow = model.material("button_yellow", rgba=(0.84, 0.68, 0.12, 1.0))
    button_blue = model.material("button_blue", rgba=(0.12, 0.40, 0.74, 1.0))

    cabinet_width = 0.42
    side_thickness = 0.012
    inner_width = cabinet_width - 2.0 * side_thickness

    profile_yz = [
        (-0.160, 0.000),
        (0.120, 0.000),
        (0.120, 0.340),
        (0.040, 0.450),
        (-0.020, 0.450),
        (-0.050, 0.320),
        (-0.072, 0.142),
        (-0.154, 0.095),
    ]

    marquee_segment = ((-0.020, 0.450), (-0.050, 0.320))
    bezel_segment = ((-0.050, 0.320), (-0.072, 0.142))
    deck_segment = ((-0.154, 0.095), (-0.072, 0.142))
    back_segment = ((0.120, 0.340), (0.040, 0.450))

    marquee_y, marquee_z, marquee_len, marquee_angle = _segment_pose(*marquee_segment)
    bezel_y, bezel_z, bezel_len, bezel_angle = _segment_pose(*bezel_segment)
    deck_y, deck_z, deck_len, deck_angle = _segment_pose(*deck_segment)
    back_y, back_z, back_len, back_angle = _segment_pose(*back_segment)

    left_side_mesh = mesh_from_geometry(
        _prism_from_profile(
            profile_yz,
            x_min=-cabinet_width * 0.5,
            x_max=-cabinet_width * 0.5 + side_thickness,
        ),
        "arcade_left_side_panel",
    )
    right_side_mesh = mesh_from_geometry(
        _prism_from_profile(
            profile_yz,
            x_min=cabinet_width * 0.5 - side_thickness,
            x_max=cabinet_width * 0.5,
        ),
        "arcade_right_side_panel",
    )

    cabinet = model.part("cabinet")
    cabinet.visual(left_side_mesh, material=body_black, name="left_side")
    cabinet.visual(right_side_mesh, material=body_black, name="right_side")
    cabinet.visual(
        Box((inner_width, 0.280, side_thickness)),
        origin=Origin(xyz=(0.000, -0.020, side_thickness * 0.5)),
        material=body_black,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((inner_width, side_thickness, 0.340)),
        origin=Origin(xyz=(0.000, 0.114, 0.170)),
        material=body_black,
        name="rear_panel_lower",
    )
    cabinet.visual(
        Box((inner_width, back_len, side_thickness)),
        origin=Origin(xyz=(0.000, back_y, back_z), rpy=(back_angle, 0.0, 0.0)),
        material=body_black,
        name="rear_panel_upper",
    )
    cabinet.visual(
        Box((inner_width, 0.060, side_thickness)),
        origin=Origin(xyz=(0.000, 0.010, 0.444)),
        material=body_black,
        name="top_panel",
    )
    cabinet.visual(
        Box((inner_width, marquee_len, side_thickness)),
        origin=Origin(xyz=(0.000, marquee_y, marquee_z), rpy=(marquee_angle, 0.0, 0.0)),
        material=body_black,
        name="marquee_panel_backer",
    )
    cabinet.visual(
        Box((inner_width * 0.96, marquee_len - 0.010, 0.004)),
        origin=Origin(
            xyz=(0.000, marquee_y - math.sin(marquee_angle) * 0.004, marquee_z + math.cos(marquee_angle) * 0.004),
            rpy=(marquee_angle, 0.0, 0.0),
        ),
        material=marquee_glow,
        name="marquee_face",
    )
    cabinet.visual(
        Box((inner_width, deck_len, side_thickness)),
        origin=Origin(xyz=(0.000, deck_y, deck_z), rpy=(deck_angle, 0.0, 0.0)),
        material=control_black,
        name="control_deck",
    )
    cabinet.visual(
        Box((inner_width, 0.095, side_thickness)),
        origin=Origin(xyz=(0.000, -0.157, 0.048), rpy=(1.06, 0.0, 0.0)),
        material=body_black,
        name="front_kick_panel",
    )

    bezel_width = 0.255
    bezel_height = 0.138
    bezel_bar = 0.016
    cabinet.visual(
        Box((inner_width, bezel_bar, 0.010)),
        origin=Origin(
            xyz=(0.000, bezel_y + 0.5 * bezel_height * math.cos(bezel_angle), bezel_z + 0.5 * bezel_height * math.sin(bezel_angle)),
            rpy=(bezel_angle, 0.0, 0.0),
        ),
        material=trim_black,
        name="bezel_top_bar",
    )
    cabinet.visual(
        Box((inner_width, bezel_bar, 0.010)),
        origin=Origin(
            xyz=(0.000, bezel_y - 0.5 * bezel_height * math.cos(bezel_angle), bezel_z - 0.5 * bezel_height * math.sin(bezel_angle)),
            rpy=(bezel_angle, 0.0, 0.0),
        ),
        material=trim_black,
        name="bezel_bottom_bar",
    )
    cabinet.visual(
        Box((bezel_bar, bezel_height, 0.010)),
        origin=Origin(xyz=(-0.5 * bezel_width, bezel_y, bezel_z), rpy=(bezel_angle, 0.0, 0.0)),
        material=trim_black,
        name="bezel_left_bar",
    )
    cabinet.visual(
        Box((bezel_bar, bezel_height, 0.010)),
        origin=Origin(xyz=(0.5 * bezel_width, bezel_y, bezel_z), rpy=(bezel_angle, 0.0, 0.0)),
        material=trim_black,
        name="bezel_right_bar",
    )
    cabinet.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.090, 0.004, 0.447)),
        material=trim_black,
        name="knob_mount",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, 0.280, 0.450)),
        mass=7.5,
        origin=Origin(xyz=(0.000, -0.020, 0.225)),
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.228, 0.128, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, -0.009)),
        material=screen_glass,
        name="screen_panel",
    )
    screen.inertial = Inertial.from_geometry(
        Box((0.228, 0.128, 0.008)),
        mass=0.28,
        origin=Origin(xyz=(0.000, 0.000, -0.009)),
    )
    model.articulation(
        "cabinet_to_screen",
        ArticulationType.FIXED,
        parent=cabinet,
        child=screen,
        origin=Origin(xyz=(0.000, bezel_y, bezel_z), rpy=(bezel_angle, 0.0, 0.0)),
    )

    knob = model.part("control_knob")
    knob.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=knob_black,
        name="knob_skirt",
    )
    knob.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.022)),
        material=knob_black,
        name="knob_cap",
    )
    knob.visual(
        Box((0.003, 0.018, 0.002)),
        origin=Origin(xyz=(0.000, 0.010, 0.023)),
        material=knob_indicator,
        name="knob_pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.023, length=0.026),
        mass=0.07,
        origin=Origin(xyz=(0.000, 0.000, 0.013)),
    )
    model.articulation(
        "cabinet_to_control_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.090, 0.004, 0.450)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=12.0),
    )

    button_surface_y, button_surface_z = _offset_along_local_z(deck_y, deck_z, deck_angle, side_thickness * 0.5)
    button_guide_y, button_guide_z = _offset_along_local_z(deck_y, deck_z, deck_angle, side_thickness * 0.5 + 0.002)
    button_specs = (
        ("red_button", "red", -0.060, button_red),
        ("yellow_button", "yellow", -0.018, button_yellow),
        ("blue_button", "blue", 0.024, button_blue),
    )
    for part_name, color_name, x_pos, material in button_specs:
        cabinet.visual(
            Cylinder(radius=0.016, length=0.004),
            origin=Origin(xyz=(x_pos, button_guide_y, button_guide_z), rpy=(deck_angle, 0.0, 0.0)),
            material=trim_black,
            name=f"{color_name}_button_guide",
        )

        button = model.part(part_name)
        button.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=material,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=0.013, length=0.006),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
        )
        model.articulation(
            f"cabinet_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, button_surface_y, button_surface_z), rpy=(deck_angle, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=0.10,
                lower=0.0,
                upper=0.003,
            ),
        )

    acceptor = model.part("coin_acceptor")
    acceptor.visual(
        Box((0.006, 0.052, 0.100)),
        origin=Origin(xyz=(0.003, 0.000, 0.050)),
        material=coin_metal,
        name="acceptor_base",
    )
    acceptor.visual(
        Box((0.008, 0.040, 0.010)),
        origin=Origin(xyz=(0.010, 0.000, 0.071)),
        material=coin_dark,
        name="coin_slot",
    )
    acceptor.visual(
        Box((0.005, 0.034, 0.020)),
        origin=Origin(xyz=(0.0085, 0.000, 0.018)),
        material=coin_dark,
        name="return_bezel",
    )
    acceptor.visual(
        Box((0.004, 0.052, 0.010)),
        origin=Origin(xyz=(0.008, 0.000, 0.095)),
        material=coin_metal,
        name="top_lip",
    )
    acceptor.inertial = Inertial.from_geometry(
        Box((0.014, 0.052, 0.100)),
        mass=0.14,
        origin=Origin(xyz=(0.007, 0.000, 0.050)),
    )
    model.articulation(
        "cabinet_to_coin_acceptor",
        ArticulationType.FIXED,
        parent=cabinet,
        child=acceptor,
        origin=Origin(xyz=(cabinet_width * 0.5, -0.114, 0.082)),
    )

    coin_flap = model.part("coin_return_flap")
    coin_flap.visual(
        Box((0.003, 0.028, 0.018)),
        origin=Origin(xyz=(0.0015, 0.000, -0.009)),
        material=coin_metal,
        name="coin_flap",
    )
    coin_flap.inertial = Inertial.from_geometry(
        Box((0.003, 0.028, 0.018)),
        mass=0.02,
        origin=Origin(xyz=(0.0015, 0.000, -0.009)),
    )
    model.articulation(
        "coin_acceptor_to_return_flap",
        ArticulationType.REVOLUTE,
        parent=acceptor,
        child=coin_flap,
        origin=Origin(xyz=(0.006, 0.000, 0.026)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.08,
            velocity=3.0,
            lower=0.0,
            upper=1.1,
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

    cabinet = object_model.get_part("cabinet")
    screen = object_model.get_part("screen")
    knob = object_model.get_part("control_knob")
    red_button = object_model.get_part("red_button")
    yellow_button = object_model.get_part("yellow_button")
    blue_button = object_model.get_part("blue_button")
    acceptor = object_model.get_part("coin_acceptor")
    coin_flap = object_model.get_part("coin_return_flap")

    knob_joint = object_model.get_articulation("cabinet_to_control_knob")
    red_button_joint = object_model.get_articulation("cabinet_to_red_button")
    yellow_button_joint = object_model.get_articulation("cabinet_to_yellow_button")
    blue_button_joint = object_model.get_articulation("cabinet_to_blue_button")
    flap_joint = object_model.get_articulation("coin_acceptor_to_return_flap")

    ctx.check("cabinet part exists", cabinet is not None)
    ctx.check("screen part exists", screen is not None)
    ctx.check("control knob part exists", knob is not None)
    ctx.check("red button part exists", red_button is not None)
    ctx.check("yellow button part exists", yellow_button is not None)
    ctx.check("blue button part exists", blue_button is not None)
    ctx.check("coin acceptor part exists", acceptor is not None)
    ctx.check("coin flap part exists", coin_flap is not None)

    ctx.check(
        "control knob uses continuous vertical rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS and knob_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )
    ctx.check(
        "control deck buttons use downward prismatic motion",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC and joint.axis == (0.0, 0.0, -1.0)
            for joint in (red_button_joint, yellow_button_joint, blue_button_joint)
        ),
        details=(
            f"red={red_button_joint.axis}/{red_button_joint.articulation_type}, "
            f"yellow={yellow_button_joint.axis}/{yellow_button_joint.articulation_type}, "
            f"blue={blue_button_joint.axis}/{blue_button_joint.articulation_type}"
        ),
    )
    ctx.check(
        "coin return flap uses horizontal revolute hinge",
        flap_joint.articulation_type == ArticulationType.REVOLUTE and flap_joint.axis == (0.0, -1.0, 0.0),
        details=f"type={flap_joint.articulation_type}, axis={flap_joint.axis}",
    )

    ctx.expect_contact(
        acceptor,
        cabinet,
        elem_a="acceptor_base",
        elem_b="right_side",
        name="coin acceptor mounts to right cabinet side",
    )
    ctx.expect_contact(
        knob,
        cabinet,
        elem_a="knob_skirt",
        elem_b="top_panel",
        name="knob sits on top panel",
    )
    for button, guide_name, label in (
        (red_button, "red_button_guide", "red"),
        (yellow_button, "yellow_button_guide", "yellow"),
        (blue_button, "blue_button_guide", "blue"),
    ):
        ctx.expect_overlap(
            button,
            cabinet,
            axes="xy",
            elem_a="button_cap",
            elem_b=guide_name,
            min_overlap=0.020,
            name=f"{label} button stays centered over its guide",
        )

    with ctx.pose({flap_joint: 0.0}):
        ctx.expect_contact(
            coin_flap,
            acceptor,
            elem_a="coin_flap",
            elem_b="return_bezel",
            name="coin flap seats against return bezel when closed",
        )

    closed_aabb = None
    open_aabb = None
    with ctx.pose({flap_joint: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(coin_flap, elem="coin_flap")
    with ctx.pose({flap_joint: 0.95}):
        open_aabb = ctx.part_element_world_aabb(coin_flap, elem="coin_flap")
    ctx.check(
        "coin flap opens outward from the side panel",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.008,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for button, joint, label in (
        (red_button, red_button_joint, "red"),
        (yellow_button, yellow_button_joint, "yellow"),
        (blue_button, blue_button_joint, "blue"),
    ):
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: joint.motion_limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{label} button depresses into the control deck",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
